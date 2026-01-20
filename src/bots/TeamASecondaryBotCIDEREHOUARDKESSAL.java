package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Bot;
import robotsimulator.Brain;

import java.util.ArrayList;
import java.util.LinkedList;

public class TeamASecondaryBotCIDEREHOUARDKESSAL extends Brain {
    // NOTE :
    /*
        Wall position :  x=0 (left) to x=2200 (right)
                         y=0 (top) to y=1800 (bottom)
     */

    /**
     * @param attr null si pas besoin
     */
    private record QueuedTask(Task task, TaskAttribute attr) {
            QueuedTask(Task task) {
                this(task, null);
            }
    }

    private static final String SB1 = "Secondary-bot-1";
    private static final String SB2 = "Secondary-bot-2";

    private enum Task {
        // MOVEMENT TASKS
        TURN_LEFT,
        TURN_RIGHT,
        CLOSE_DISTANCE,

        // FORMATION TASKS
        GET_INTO_FORMATION,

        // MESSAGE TASKS
        ENEMY_DETECTED_AND_MARKED,

        // ATTACK TASKS
        ATTACK_NEAREST_ENEMY, MOVE_FORWARD, TURN_TOWARD_TARGET, WAITING_FOR_ALLY_STATUS,

        ROAM_AND_AVOID_ATTACKS, MOVE_A_BIT, MOVE_BACK_A_BIT, TURN // Robot perdu, se déplace aléatoirement pour trouver un allié
    }

    private static class TaskAttribute {
        double targetHeading;
        private int currentStep;
        private int targetWaitingSteps;

        TaskAttribute(double targetHeading) {
            this.targetHeading = targetHeading;
        }

        TaskAttribute(int targetWaitingSteps) {
            this.targetWaitingSteps = targetWaitingSteps;
        }

        public boolean isTargetWaitingStepsReached() {
            return currentStep >= targetWaitingSteps;
        }

        public void incrementWaitingStep() {
            this.currentStep++;
        }
    }

    private static final double TURN_HYSTERESIS = 0.05;
    private static final double ANGLE_PRECISION = 0.01;
    private final static double HEADING_PRECISION = 0.01;

    // Constantes du terrain
    private static final double ARENA_WIDTH = 3000.0;
    private static final double ARENA_HEIGHT = 2000.0;
    private static final double BOT_RADIUS = Parameters.teamASecondaryBotRadius;
    private static final double FRONT_SENSOR_RANGE = Parameters.teamASecondaryBotFrontalDetectionRange;

    private LinkedList<QueuedTask> taskQueue;

    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    private Bot bot;

    @Override
    public void activate() {
        myID = SB1;
        for (IRadarResult o: detectRadar())
            if (isSameDirection(o.getObjectDirection(),Parameters.NORTH)) myID=SB2;
        if (myID.equals(SB1)){
            myX=Parameters.teamASecondaryBot1InitX;
            myY=Parameters.teamASecondaryBot1InitY;
        } else {
            myX=Parameters.teamASecondaryBot2InitX;
            myY=Parameters.teamASecondaryBot2InitY;
        }

        taskQueue = new LinkedList<>();
        taskQueue.addFirst(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
    }

    @Override
    public void bind(Bot bot) {
        this.bot = bot;
        super.bind(bot);
    }

    private void updateOdometryAfterMove(boolean forward) {
        if (forward) {
            myX = this.myX + Parameters.teamASecondaryBotSpeed * Math.cos(this.getHeading());
            myY = this.myY + Parameters.teamASecondaryBotSpeed * Math.sin(this.getHeading());
        } else {
            myX = this.myX - Parameters.teamASecondaryBotSpeed * Math.cos(this.getHeading());
            myY = this.myY - Parameters.teamASecondaryBotSpeed * Math.sin(this.getHeading());
        }
    }

    public void callNextTask() {
        if (!taskQueue.isEmpty()) {
            switch (taskQueue.getFirst().task) {
                case MOVE_BACK_A_BIT:
                    moveBackAbit();
                    break;
                case MOVE_A_BIT:
                    moveAbit();
                    break;
                case TURN :
                    turn();
                    break;
                case MOVE_FORWARD:
                    moveForward();
                    break;
                case TURN_LEFT:
                    turnLeft();
                    break;
                case TURN_RIGHT:
                    turnRight();
                    break;
                case ROAM_AND_AVOID_ATTACKS:
                    roamAndAvoidAttacksBehavior();
                    break;
            }
        }
    }

    private static int MAP_WIDTH = 3000;
    private static int MAP_HEIGHT = 2000;

    // if forward is true, check forward displacement, else backward
    public boolean isDisplacementPossible(boolean forward) {
        double newX;
        double newY;
        if (forward) {
            newX = myX + Parameters.teamASecondaryBotSpeed * Math.cos(getHeading());
            newY = myY + Parameters.teamASecondaryBotSpeed * Math.sin(getHeading());
        } else {
            newX = myX - Parameters.teamASecondaryBotSpeed * Math.cos(getHeading());
            newY = myY - Parameters.teamASecondaryBotSpeed * Math.sin(getHeading());
        }
        return (newX >= Parameters.teamASecondaryBotRadius && newX <= MAP_WIDTH - Parameters.teamASecondaryBotRadius && newY >= Parameters.teamASecondaryBotRadius && newY <= (double)MAP_HEIGHT - Parameters.teamASecondaryBotRadius);
    }

    public void myMove() {
        if (isDisplacementPossible(true)) {
            if (taskQueue.getFirst().task == Task.MOVE_A_BIT) {
                // Bypass object too close check when moving a bit
                updateOdometryAfterMove(true);
                move();
                return;
            }

            if (isObjectTooClose()) {
                sendLogMessage("Object too close! Turn away and move.");
                return;
            }

            if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
                sendLogMessage("Wall detected ahead! Stopping movement.");
                return;
            }

            updateOdometryAfterMove(true);
            move();
        }
    }

    private boolean isObjectTooCloseBehind() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.BULLET) continue;
            if (obj.getObjectDistance() <= (Parameters.teamASecondaryBotRadius + obj.getObjectRadius()) * 1.4) {
                // Ally too close behind, turn left or right then move a bit forward
                taskQueue.clear();
                taskQueue.addFirst(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
                return true;
            }
        }
        return false;
    }

    public void myMoveBack() {
        if (!isDisplacementPossible(false)) return;

        // Check if an object is too close behind
        if (isObjectTooCloseBehind()) {
            sendLogMessage("Object too close behind! Turning away and moving.");
            return;
        }

        updateOdometryAfterMove(false);
        moveBack();
    }

    private boolean doTaskQueueContains(Task task){
        for (QueuedTask qt: taskQueue){
            if (qt.task==task) return true;
        }
        return false;
    }

    // Help determining optimal turn direction to avoid obstacle
    private double getOptimalTurnDirection() {
        IRadarResult closestObject = getRadarClosestObject();
        if (closestObject == null) {
            return 0; // No object detected
        }
        double objectDirection = closestObject.getObjectDirection();

        // If object is on the right, turn left; if on the left, turn right
        double sideStep = (objectDirection >= 0) ? -Math.PI / 2 : Math.PI / 2;

        return getHeading() + sideStep;
    }

    private IRadarResult getRadarClosestObject() {
        IRadarResult closestObject = null;
        double minDistance = Double.MAX_VALUE;
        for (IRadarResult obj : detectRadar()) {
            double distance = obj.getObjectDistance();
            if (distance < minDistance) {
                minDistance = distance;
                closestObject = obj;
            }
        }
        return closestObject;
    }

    private boolean isObjectTooClose() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.BULLET) continue;
            if (obj.getObjectDistance() <= (Parameters.teamASecondaryBotRadius + obj.getObjectRadius()) * 2) {
                // Ally too close, move a bit backward, then turn away
                double targetHeading = getOptimalTurnDirection();
                taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(targetHeading)));
                taskQueue.addFirst(new QueuedTask(Task.MOVE_BACK_A_BIT, new TaskAttribute(100)));
                return true;
            }
        }
        return false;
    }

    public void step() {
        if (getHealth() <= 0) {
            return;
        }
        if (!currentlyAvoidingEnnemy)
            checkForEnemiesAndRespond();

        // Execute current task if any
        if (!taskQueue.isEmpty()) {
            callNextTask();
        }
    }

    // LONGEMENT DES MURS
    private void turnLeft() {
        // Check if the turn is complete
        if (isHeadingReached(taskQueue.getFirst().attr.targetHeading)) {
            taskQueue.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.LEFT);
        }
    }

    private void turnRight() {
        // Check if the turn is complete
        if (isHeadingReached(taskQueue.getFirst().attr.targetHeading)) {
            taskQueue.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.RIGHT);
        }
    }

    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

    // Returns null if no turn is needed (inside hysteresis), else returns optimal turn direction
    private Parameters.Direction getOptimalTurnDirectionWithHysteresis(double targetDirection) {
        double diff = targetDirection - getHeading();
        // Normalize diff to [-PI, PI]
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;

        if (Math.abs(diff) < TURN_HYSTERESIS) {
            return null;
        }
        return diff < 0 ? Parameters.Direction.LEFT : Parameters.Direction.RIGHT;
    }

    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(dir1-dir2)< ANGLE_PRECISION;
    }

    public void moveBackAbit() {
        TaskAttribute currentTask = taskQueue.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            taskQueue.removeFirst();
            sendLogMessage("Move back a bit complete.");
            callNextTask();
        } else {
            myMoveBack();
            currentTask.incrementWaitingStep();
        }
    }

    // RENDEZ-VOUS logic
    // Checking for messages
    private boolean receiveMessage(String message) {
        String[] parts = message.split(";");
        // NO MESSAGE YET
        return false;
    }

    // Move forward until an obstacle is detected
    public void moveForward() {
        IFrontSensorResult frontSensorResult = this.detectFront();
        if (frontSensorResult.getObjectType().equals(IFrontSensorResult.Types.NOTHING)) {
            myMove();
        } else {
            // Obstacle detected, stop moving
            taskQueue.removeFirst();
            sendLogMessage("Obstacle detected ahead. Stopping movement.");
            // Turn around
            double turnDirection = getHeading() + Math.PI + (Math.random() - 0.5) * Math.PI; // [-π/2, +π/2] autour de derrière
            taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(turnDirection)));
        }
    }

    private void moveAbit() {
        TaskAttribute currentTask = taskQueue.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            // Move complete
            taskQueue.removeFirst();
            sendLogMessage("Move a bit complete.");
            callNextTask();
        } else {
            myMove();
            currentTask.incrementWaitingStep();
        }
    }

    private static final String FIGHTING_ENEMY_MESSAGE = "FIGHTING_ENEMY";
    private boolean currentlyAvoidingEnnemy = false;

    public void checkForEnemiesAndRespond() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                // Enemy detected
                double enemyDirection = result.getObjectDirection();
                double absAngle = result.getObjectDirection();
                double enemyDistance = result.getObjectDistance();
                // Sends location to allie bots

                double enemyX = myX + enemyDistance * Math.cos(absAngle);
                double enemyY = myY + enemyDistance * Math.sin(absAngle);
                String enemyMessage = FIGHTING_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID;
                broadcast(enemyMessage);
                //System.out.println(myID + " currently at (" + myX + ", " + myY + ") while actual position is : " + (bot.getX()) + ", " + (bot.getY()));
                //System.out.println(myID + " detected enemy at (" + enemyX + ", " + enemyY + ") and sent message.");

                // 1/2 chance to turn perpendicularly left or right
                double awayDirection = enemyDirection + Math.PI + (Math.random() - 0.5) * Math.PI; // ∈ [-π/2, +π/2]

                Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(awayDirection);
                if (turnDirection != null) {
                    taskQueue.clear();
                    taskQueue.addFirst(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
                    taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
                    taskQueue.add(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(20)));
                    taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(awayDirection)));
                    currentlyAvoidingEnnemy = true;
                }
            }
        }
    }

    // Roam, detect ennemies, then run back to spawn and retry
    public void roamAndAvoidAttacksBehavior() {
        // Checking for allied bot in radar
        currentlyAvoidingEnnemy = false;
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    result.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                // Allied bot detected
                // Check if we are far enough
                double distance = result.getObjectDistance();
                if (distance < (Parameters.teamASecondaryBotRadius + result.getObjectRadius()) * 1.1) {
                    // Too close turn away
                    double awayDirection = result.getObjectDirection() + Math.PI;
                    Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(awayDirection);
                    if (turnDirection != null) {
                        taskQueue.addFirst((new QueuedTask(Task.TURN_LEFT, new TaskAttribute(awayDirection))));
                        return;
                    }
                }
            }
        }

        // Move randomly
        taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
        // Generate a random turn
        double randomTurnAngle = Math.random() * 2 * Math.PI;
        taskQueue.add(new QueuedTask(Task.TURN, new TaskAttribute(randomTurnAngle)));
    }

    private void turn() {
        TaskAttribute currentTask = taskQueue.getFirst().attr;
        double targetHeading = currentTask.targetHeading;
        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(targetHeading);
        if (turnDirection != null) {
            stepTurn(turnDirection);
        } else {
            // Turn complete
            taskQueue.removeFirst();
            sendLogMessage("Turn complete.");
            callNextTask();
        }
    }
}
