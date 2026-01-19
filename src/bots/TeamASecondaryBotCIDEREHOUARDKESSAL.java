package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
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

        ROAM_AND_AVOID_ATTACKS, MOVE_A_BIT, TURN // Robot perdu, se déplace aléatoirement pour trouver un allié
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

    // ===================== ODOMETRY CORRECTION =====================

    private void correctOdometryWithWalls() {
        IFrontSensorResult frontSensor = detectFront();
        if (frontSensor.getObjectType() != IFrontSensorResult.Types.WALL) {
            return;
        }

        double heading = getHeading();
        while (heading < 0) heading += 2 * Math.PI;
        while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;

        double distanceToWall = FRONT_SENSOR_RANGE;
        double tolerance = Math.PI / 6;

        if (Math.abs(heading - 0) < tolerance || Math.abs(heading - 2*Math.PI) < tolerance) {
            double correctedX = ARENA_WIDTH - distanceToWall - BOT_RADIUS;
            if (Math.abs(myX - correctedX) > 50) {
                myX = correctedX;
            }
        }
        else if (Math.abs(heading - Math.PI/2) < tolerance) {
            double correctedY = ARENA_HEIGHT - distanceToWall - BOT_RADIUS;
            if (Math.abs(myY - correctedY) > 50) {
                myY = correctedY;
            }
        }
        else if (Math.abs(heading - Math.PI) < tolerance) {
            double correctedX = distanceToWall + BOT_RADIUS;
            if (Math.abs(myX - correctedX) > 50) {
                myX = correctedX;
            }
        }
        else if (Math.abs(heading - 3*Math.PI/2) < tolerance || Math.abs(heading + Math.PI/2) < tolerance) {
            double correctedY = distanceToWall + BOT_RADIUS;
            if (Math.abs(myY - correctedY) > 50) {
                myY = correctedY;
            }
        }
    }

    private void clampPositionToArena() {
        double oldX = myX, oldY = myY;

        if (myX < BOT_RADIUS)
            myX = BOT_RADIUS;
        if (myX > ARENA_WIDTH - BOT_RADIUS)
            myX = ARENA_WIDTH - BOT_RADIUS;
        if (myY < BOT_RADIUS)
            myY = BOT_RADIUS;
        if (myY > ARENA_HEIGHT - BOT_RADIUS)
            myY = ARENA_HEIGHT - BOT_RADIUS;

        if (oldX != myX || oldY != myY) {
        }
    }

    private void updateOdometryAfterMove() {
        myX = this.myX + Parameters.teamASecondaryBotSpeed * Math.cos(this.getHeading());
        myY = this.myY + Parameters.teamASecondaryBotSpeed * Math.sin(this.getHeading());

        clampPositionToArena();
    }

    public void callNextTask() {
        if (!taskQueue.isEmpty()) {
            switch (taskQueue.getFirst().task) {
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

    public void myMove() {

        if (doTaskQueueContains(Task.MOVE_A_BIT)){
            sendLogMessage("Another MOVE_A_BIT in queue, not moving to avoid conflicts.");
            updateOdometryAfterMove();
            move();
            return;
        }

        if (isAllyTooClose()) {
            sendLogMessage("Ally too close! Turn away and move.");
            return;
        }

        if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
            sendLogMessage("Wall detected ahead! Stopping movement.");
            return; // Do not move if obstacle ahead
        }

        updateOdometryAfterMove();
        move();
    }

    private boolean doTaskQueueContains(Task task){
        for (QueuedTask qt: taskQueue){
            if (qt.task==task) return true;
        }
        return false;
    }


    private boolean isAllyTooClose() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot
                    || obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {

                if (obj.getObjectDistance() <= (Parameters.teamASecondaryBotRadius + obj.getObjectRadius()) * 1.5) {
                    // Ally too close, turn away
                    // Check if bot should be moving a bit
                    for (QueuedTask task : taskQueue) {
                        if (task.task == Task.MOVE_A_BIT) {
                            taskQueue.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(5)));
                            sendLogMessage("trying to move a bit to avoid ally");
                            return true;
                        }
                    }
                    double turnDirection = obj.getObjectDirection() + Math.PI; // Turn away
                    taskQueue.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(2)));
                    taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(turnDirection)));
                    return true;
                }
            }
        }
        return false;
    }

    public void step() {
        if (getHealth() <= 0) {
            return;
        }

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

    public void checkForEnemiesAndRespond() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                // Enemy detected
                double enemyDirection = result.getObjectDirection();
                double enemyDistance = result.getObjectDistance();
                // Sends location to allie bots
                double enemyX = myX + enemyDistance * Math.cos(enemyDirection);
                double enemyY = myY + enemyDistance * Math.sin(enemyDirection);
                String enemyMessage = FIGHTING_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID;
                broadcast(enemyMessage);

                // 1/2 chance to turn perpendicularly left or right
                double awayDirection = enemyDirection + Math.PI/2;

                Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(awayDirection);
                if (turnDirection != null) {
                    taskQueue.clear();
                    taskQueue.addFirst(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
                    taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
                    taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(awayDirection)));
                    taskQueue.add(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(10)));
                }
            }
        }
    }

    // Roam, detect ennemies, then run back to spawn and retry
    public void roamAndAvoidAttacksBehavior() {
        // Checking for allied bot in radar
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    result.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                // Allied bot detected
                // Check if we are far enough
                double distance = result.getObjectDistance();
                if (distance < (Parameters.teamASecondaryBotRadius + result.getObjectRadius()) * 1.1) {
                    // Too close turn away
                    double awayDirection = myID.equals(SB1) ? result.getObjectDirection() + Math.PI / 2 : result.getObjectDirection() + Math.PI / 2;
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
