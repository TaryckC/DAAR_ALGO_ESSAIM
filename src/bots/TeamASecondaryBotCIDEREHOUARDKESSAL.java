package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;
import java.util.LinkedList;

public class TeamASecondaryBotCIDEREHOUARDKESSAL extends Brain {

    private static final String SB1 = "Secondary-bot-1";
    private static final String SB2 = "Secondary-bot-2";

    private static final String FIGHTING_ENEMY_MESSAGE = "FIGHTING_ENEMY";

    private static final int MAP_WIDTH = 3000;
    private static final int MAP_HEIGHT = 2000;

    private static final double TURN_HYSTERESIS = 0.05;
    private static final double ANGLE_PRECISION = 0.01;
    private final static double HEADING_PRECISION = 0.01;

    private boolean currentlyAvoidingEnemy = false;

    private enum Task {
        TURN_LEFT,
        TURN_RIGHT,
        MOVE_FORWARD,
        ROAM_AND_AVOID_ATTACKS, MOVE_A_BIT, MOVE_BACK_A_BIT, TURN
    }

    private record QueuedTask(Task task, TaskAttribute attr) {
        QueuedTask(Task task) {
            this(task, null);
        }
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

    private LinkedList<QueuedTask> taskQueue;

    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    // STARTS BOT
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

    // ===================== DISPLACEMENT ODOMETRY =====================

    private void updateOdometryAfterMove(boolean forward) {
        if (forward) {
            myX = this.myX + Parameters.teamASecondaryBotSpeed * Math.cos(this.getHeading());
            myY = this.myY + Parameters.teamASecondaryBotSpeed * Math.sin(this.getHeading());
        } else {
            myX = this.myX - Parameters.teamASecondaryBotSpeed * Math.cos(this.getHeading());
            myY = this.myY - Parameters.teamASecondaryBotSpeed * Math.sin(this.getHeading());
        }
    }

    // ===================== TASK GESTION =====================

    // Call the next task in the queue
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

    // ===================== VERIFICATION =====================

    // Check if displacement is possible (no obstacle in the way and inside map boundaries)
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

        boolean cond1 = (newX >= Parameters.teamASecondaryBotRadius && newX <= MAP_WIDTH - Parameters.teamASecondaryBotRadius && newY >= Parameters.teamASecondaryBotRadius && newY <= (double)MAP_HEIGHT - Parameters.teamASecondaryBotRadius);

        for(IRadarResult obstacle : detectRadar()) {
            if (!(obstacle.getObjectType() == IRadarResult.Types.BULLET)) {
                double absAngle = obstacle.getObjectDirection();
                double enemyDistance = obstacle.getObjectDistance();

                double obstacleX = myX + enemyDistance * Math.cos(absAngle);
                double obstacleY = myY + enemyDistance * Math.sin(absAngle);
                boolean cond2 = ((newX - obstacleX) * (newX - obstacleX) + (newY - obstacleY) * (newY - obstacleY) < (Parameters.teamASecondaryBotRadius + obstacle.getObjectRadius()) * (Parameters.teamASecondaryBotRadius + obstacle.getObjectRadius()));
                if (cond2) {
                    return false;
                }
            }
        }
        return cond1;
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

    // Check if an object is too close in front
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

    // Check for enemies in radar and respond accordingly
    // We could've also tried and detect enemy using front sensor
    public void checkForEnemiesAndRespond() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        // Get Nearest enemy and broadcast its position
        IRadarResult result = getNearestEnemy(radarResults, 0);
        if (result != null) {
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
                currentlyAvoidingEnemy = true;
            }
        }
    }

    // ===================== ACTIONS =====================

    // add guards to movement (forward), update odometry after (actually before) move
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

    // Move forward until target waiting steps reached
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

    // add guards to movement (backward), update odometry after (actually before) move
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

    // Move backward until target waiting steps reached
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

    // Advance in a straight line while sometimes turning randomly, also avoids allied bots
    public void roamAndAvoidAttacksBehavior() {
        // Checking for allied bot in radar
        currentlyAvoidingEnemy = false;
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

    // Turn towards target heading (given by a task attribute)
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

    // ===================== UTILS FUNCTION =====================

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

    // Use heading precision to determine if heading is reached
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

    // Use angle precision to determine if two directions are the same (approx
    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(dir1-dir2)< ANGLE_PRECISION;
    }

    // Check if enemy is of specified type (0: all, 1: main bot only, 2: secondary bot only)
    private boolean isEnemyOfType(IRadarResult enemy, int type) {
        return switch (type) {
            case 0 ->
                    enemy.getObjectType() == IRadarResult.Types.OpponentMainBot || enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
            case 1 -> enemy.getObjectType() == IRadarResult.Types.OpponentMainBot;
            case 2 -> enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
            default -> false;
        };
    }

    // Get nearest enemy of specified type (0: all, 1: main bot only, 2: secondary bot only)
    private IRadarResult getNearestEnemy(ArrayList<IRadarResult> enemies, int type) {
        IRadarResult nearestEnemy = null;
        double minDistance = Double.MAX_VALUE;
        for (IRadarResult enemy : enemies) {
            if (!isEnemyOfType(enemy, type)) {
                continue;
            }
            double currentDistance = enemy.getObjectDistance();
            if (currentDistance < minDistance) {
                minDistance = currentDistance;
                nearestEnemy = enemy;
            }
        }
        return nearestEnemy;
    }

    // ===================== MAIN LOOP =====================

    public void step() {
        if (getHealth() <= 0) {
            return;
        }

        if (!currentlyAvoidingEnemy)
            checkForEnemiesAndRespond();

        // Execute current task if any
        if (!taskQueue.isEmpty()) {
            callNextTask();
        }
    }
}
