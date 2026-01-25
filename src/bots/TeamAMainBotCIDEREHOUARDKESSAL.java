package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;

import java.util.ArrayList;
import java.util.LinkedList;

public class TeamAMainBotCIDEREHOUARDKESSAL extends Brain {

    private static final String MB1 = "Main-bot-1";
    private static final String MB2 = "Main-bot-2";
    private static final String MB3 = "Main-bot-3";

    private static final String FIGHTING_ENEMY_MESSAGE = "FIGHTING_ENEMY";

    private static final double TURN_HYSTERESIS = 0.2;
    private static final double ANGLE_PRECISION = 0.2;
    private final static double HEADING_PRECISION = 0.01;

    private static final int MAP_WIDTH = 3000;
    private static final int MAP_HEIGHT = 2000;


    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    private final String allyAskingForHelpID = null;
    private double targetX;
    private double targetY;

    private enum Task {
        TURN_LEFT,
        TURN_RIGHT,
        CLOSE_DISTANCE,
        GO_AROUND_OBJECT,
        TURN,
        MOVE_A_BIT,
        COVER_AREA,
        SHOOT_AND_ADVANCE,
        SHOOT_AND_HELP,
        STOP_AND_SHOOT, MOVE_BACK_A_BIT
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
        private double targetDistance;

        TaskAttribute(double targetHeading) {
            this.targetHeading = targetHeading;
        }

        TaskAttribute(int targetWaitingSteps) {
            this.targetWaitingSteps = targetWaitingSteps;
        }

        TaskAttribute(double targetDirection, double targetDistance, int targetWaitingSteps) {
            this.targetHeading = targetDirection;
            this.targetDistance = targetDistance;
            this.targetWaitingSteps = targetWaitingSteps;
        }

        public double getShootingDirection() {
            return targetHeading;
        }

        public double getDistanceToEnemy() {
            return targetDistance;
        }

        String allyId;

        TaskAttribute(String allyId) {
            this.allyId = allyId;
        }
        public boolean isTargetWaitingStepsReached() {
            return currentStep >= targetWaitingSteps;
        }

        public void incrementWaitingStep() {
            this.currentStep++;
        }
    }

    private LinkedList<QueuedTask> currentTasks;

    // to alternate shooting and advancing
    private int shootAndAdvanceCounter = 0;

    // to disable shooting at the beginning
    private int waitBeforeShooting = 0;
    private static final int WAIT_STEPS_BEFORE_SHOOTING = 750;

    // STARTS BOT
    @Override
    public void activate() {
        // Determine my ID based on detected allied main bots
        boolean isAlliedBotUpwardDetected = false;
        boolean isAlliedBotDownwardDetected = false;
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot) {
                double direction = obj.getObjectDirection();
                if (isSameDirection(direction, Parameters.SOUTH)) {
                    isAlliedBotDownwardDetected = true;
                } else if (isSameDirection(direction, Parameters.NORTH)) {
                    isAlliedBotUpwardDetected = true;
                }
            }
        }

        if (isAlliedBotUpwardDetected && isAlliedBotDownwardDetected) {
            myID = MB2;
        } else if (isAlliedBotUpwardDetected) {
            myID = MB1;
        } else {
            myID = MB3;
        }

        switch (myID) {
            case MB3:
                myX = Parameters.teamAMainBot1InitX;
                myY = Parameters.teamAMainBot1InitY;
                break;
            case MB2:
                myX = Parameters.teamAMainBot2InitX;
                myY = Parameters.teamAMainBot2InitY;
                break;
            case MB1:
                myX = Parameters.teamAMainBot3InitX;
                myY = Parameters.teamAMainBot3InitY;
                break;
        }
        // mains bots have no initial task to complete
        // ... waiting for messages from allies
        currentTasks = new LinkedList<>();
    }

    // ===================== DISPLACEMENT ODOMETRY =====================

    // Checks for possibility of the displacement are to be made before calling this function

    // Updates odometry after a move (forward or backward)
    private void updateOdometryAfterMove(boolean forward) {
        if (forward) {
            myX = this.myX + Parameters.teamAMainBotSpeed * Math.cos(this.getHeading());
            myY = this.myY + Parameters.teamAMainBotSpeed * Math.sin(this.getHeading());
        } else {
            myX = this.myX - Parameters.teamAMainBotSpeed * Math.cos(this.getHeading());
            myY = this.myY - Parameters.teamAMainBotSpeed * Math.sin(this.getHeading());
        }
    }

    // ===================== TASK GESTION =====================

    // Call the next task in the queue
    public void callNextTask() {
        if (!currentTasks.isEmpty()) {
            switch (currentTasks.getFirst().task) { // replaced getFirst() -> get(0)
                case STOP_AND_SHOOT:
                    stopAndShoot();
                    break;
                case MOVE_BACK_A_BIT:
                    moveBack();
                    break;
                case SHOOT_AND_HELP :
                    shootAndHelp();
                    break;
                case SHOOT_AND_ADVANCE:
                    shootAndAdvance();
                    break;
                case GO_AROUND_OBJECT:
                    goAroundObject();
                    break;
                case MOVE_A_BIT :
                    moveAbit();
                    break;
                case TURN :
                    turn();
                    break;
                case TURN_LEFT:
                    turnLeft();
                    break;
                case TURN_RIGHT:
                    turnRight();
            }
        }
    }

    // Check if current tasks contain a specific task
    private boolean doCurrentTaskContain(Task obj) {
        for (QueuedTask task : currentTasks) {
            if (task.task == obj) {
                return true;
            }
        }
        return false;
    }

    // ===================== VERIFICATION =====================

    // Check if displacement is possible in the given direction (forward or backward)
    public boolean isDisplacementPossible(boolean forward) {
        double newX;
        double newY;
        if (forward) {
            newX = myX + Parameters.teamAMainBotSpeed * Math.cos(getHeading());
            newY = myY + Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        } else {
            newX = myX - Parameters.teamAMainBotSpeed * Math.cos(getHeading());
            newY = myY - Parameters.teamAMainBotSpeed * Math.sin(getHeading());
        }
        boolean cond1 = (newX >= Parameters.teamAMainBotRadius && newX <= MAP_WIDTH - Parameters.teamAMainBotRadius && newY >= Parameters.teamAMainBotRadius && newY <= (double)MAP_HEIGHT - Parameters.teamAMainBotRadius);

        for(IRadarResult obstacle : detectRadar()) {
            if (!(obstacle.getObjectType() == IRadarResult.Types.BULLET)) {
                double angle = obstacle.getObjectDirection();
                double enemyDistance = obstacle.getObjectDistance();

                double obstacleX = myX + enemyDistance * Math.cos(angle);
                double obstacleY = myY + enemyDistance * Math.sin(angle);
                boolean cond2 = ((newX - obstacleX) * (newX - obstacleX) + (newY - obstacleY) * (newY - obstacleY) < (Parameters.teamAMainBotRadius + obstacle.getObjectRadius()) * (Parameters.teamAMainBotRadius + obstacle.getObjectRadius()));
                if (cond2) {
                    return false;
                }
            }
        }
        return cond1;
    }

    // Check if current heading is close enough to target
    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

    // Check if there's an object too close behind
    private boolean isObjectTooCloseBehind() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.BULLET) continue;
            if (obj.getObjectDistance() <= (Parameters.teamAMainBotRadius + obj.getObjectRadius()) * 1.1) {
                return true;
            }
        }
        return false;
    }

    // Check if there's an obstacle too close in front
    private boolean isObstacleTooClose(double factor) {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot) {
                double distance = obj.getObjectDistance();
                double radius = Parameters.teamAMainBotRadius;
                if (distance < factor * (Parameters.teamAMainBotRadius + radius)) {
                    return true;
                }
            } else if (obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                double distance = obj.getObjectDistance();
                double radius = Parameters.teamASecondaryBotRadius;
                if (distance < factor * (Parameters.teamAMainBotRadius + radius)) {
                    return true;
                }
            } else if (obj.getObjectType() == IRadarResult.Types.Wreck) {
                double distance = obj.getObjectDistance();
                double radius = Parameters.teamAMainBotRadius;
                if (distance < factor * (Parameters.teamAMainBotRadius + radius)) {
                    return true;
                }
            }
        }
        return false;
    }

    // ===================== ACTIONS =====================

    // Move forward a bit while checking for obstacles ahead
    private void myMove() {
        if (isDisplacementPossible(true)) {
            if (currentTasks.getFirst().task == Task.MOVE_A_BIT) {
                updateOdometryAfterMove(true);
                move();
                return;
            }

            // Check if facing wall
            if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
                avoidWall();
                return;
            }

            if (isObstacleTooClose(1.04)) {
                sendLogMessage("Object too close: initiating go around maneuver.");
                currentTasks.addFirst(new QueuedTask(Task.GO_AROUND_OBJECT));
                callNextTask();
                return;
            }
            updateOdometryAfterMove(true);
            move();
        }
    }

    // Move forward a bit while checking for obstacles ahead
    private void moveAbit() {
        TaskAttribute currentTask = currentTasks.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            // Move complete
            currentTasks.removeFirst();
            sendLogMessage("Move a bit complete.");
            System.out.println( "[" + myID + "] : "+ "Move a bit complete. calling next task : " + (currentTasks.isEmpty() ? "none" : currentTasks.getFirst().task));
            callNextTask();
        } else {
            // If we're still blocked, change heading a bit and retry instead of ramming the wreck forever
            IFrontSensorResult front = detectFront();
            if (front.getObjectType() == IFrontSensorResult.Types.Wreck) {
                // Abort current MOVE_A_BIT and perform a small additional turn away
                currentTasks.removeFirst();
                double delta = (Math.random() < 0.5) ? (Math.PI / 6) : (-Math.PI / 6);
                currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(30)));
                currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(normalizeAngle(getHeading() + delta))));
                sendLogMessage("Blocked during MOVE_A_BIT: adjusting heading.");
                return;
            }

            myMove();
            currentTask.incrementWaitingStep();
        }
    }

    // Move back a bit while checking for obstacles behind
    public void moveBack() {
        // Check if the move back is complete
        TaskAttribute currentTask = currentTasks.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            // Move complete
            currentTasks.removeFirst();
            sendLogMessage("Move back a bit complete.");
            callNextTask();
        } else {
            // Check surroundings before moving back
            for (IRadarResult obj : detectRadar()) {
                double angleToObj = normalizeAngle(obj.getObjectDirection());
                double angleDiff = normalizeAngle(angleToObj - Math.PI);
                double lateral = obj.getObjectDistance() * Math.sin(angleDiff);
                double forward = obj.getObjectDistance() * Math.cos(angleDiff);

                if (forward > 0 && Math.abs(lateral) < Parameters.teamAMainBotRadius + obj.getObjectRadius() + 5) {
                    sendLogMessage("Obstacle detected behind: aborting move back.");
                    // Abort move back
                    currentTasks.removeFirst();
                    callNextTask();
                    return;
                }
            }
            currentTask.incrementWaitingStep();
            myMoveBack();
        }
    }

    // Move back while checking for obstacles
    private void myMoveBack() {
        if (isDisplacementPossible(false)) {
            // Check if there's space to move back
            if (isObjectTooCloseBehind()) {
                sendLogMessage("Object too close behind: aborting move back.");
                // Abort move back
                currentTasks.removeFirst();
                callNextTask();
                return;
            }

            // Update odométrie before moving back
            updateOdometryAfterMove(false);
            moveBack();
        }
    }

    // Head toward target coordinates in a straight line (hopefully)
    private void headTowardCoord() {
        double dx = targetX - myX;
        double dy = targetY - myY;
        double distance = Math.sqrt((myX - targetX) * (myX - targetX) + (myY - targetY) * (myY - targetY));
        double directionToTarget = Math.atan2(dy, dx);

        if (distance <= Parameters.bulletRange - 100) {

            sendLogMessage("Reached target coordinates (" + targetX + ", " + targetY + ").");
            if (currentTasks.getFirst().task == Task.SHOOT_AND_HELP) {
                // Aim at the given coordinates and shoot
                currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
                currentTasks.addFirst(new QueuedTask(Task.STOP_AND_SHOOT, new TaskAttribute(directionToTarget, distance, 200)));
                currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(directionToTarget)));
                System.out.println("[" + myID + "]" + "Shooting toward : " + directionToTarget + " distance " + distance);
                // Turn toward the target coordinates
            }
            return;
        }

        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(directionToTarget);
        if (turnDirection != null) {
            stepTurn(turnDirection);
        } else {
            myMove();
        }
    }

    // Maneuver to go around the closest detected object
    private void goAroundObject() {
        // remove GO_AROUND_OBJECT task
        currentTasks.removeFirst();

        // Determine nearest object position relative to us
        IRadarResult closestObject = getRadarClosestObject();
        if (closestObject == null) {
            sendLogMessage("No object detected: stopping go around maneuver.");
            return;
        }
        // objectDirection is RELATIVE to our current heading
        double targetHeading = getOptimalTurnDirection();

        currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(40)));
        currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(targetHeading)));
        currentTasks.addFirst(new QueuedTask(Task.MOVE_BACK_A_BIT, new TaskAttribute(60)));
    }

    private void turn() {
        TaskAttribute currentTask = currentTasks.getFirst().attr;
        double targetHeading = currentTask.targetHeading;
        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(targetHeading);
        if (turnDirection != null) {
            stepTurn(turnDirection);
        } else {
            // Turn complete
            currentTasks.removeFirst();
            sendLogMessage("Turn complete.");
            callNextTask();
        }
    }

    private void turnLeft() {
        if (isHeadingReached(currentTasks.getFirst().attr.targetHeading)) {
            // If aligned, task complete, remove it and call next task
            currentTasks.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.LEFT);
        }
    }

    private void turnRight() {
        if (isHeadingReached(currentTasks.getFirst().attr.targetHeading)) {
            // If aligned, task complete, remove it and call next task
            currentTasks.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.RIGHT);
        }
    }

    // Attacks nearest enemy detected (either in front or by radar)
    private boolean attackNearestEnemy() {
        // If enemy in sight, shoot it
        IFrontSensorResult front = detectFront();
        if (front.getObjectType() == IFrontSensorResult.Types.OpponentMainBot ||
                front.getObjectType() == IFrontSensorResult.Types.OpponentSecondaryBot) {
            myFire(getHeading(), -1, true);
            return true;
        }

        // Or shoot at nearest enemy detected by radar
        ArrayList<IRadarResult> objects = this.detectRadar();
        IRadarResult nearestEnemy = getNearestEnemy(objects, 0);
        if (nearestEnemy != null) {
            // Suppose there's no ally in front of us
            double direction = nearestEnemy.getObjectDirection();
            double distance = nearestEnemy.getObjectDistance();
            myFire(direction, distance, true);
            return true;
        }
        return false;
    }

    // Shoots randomly toward target direction while moving to help allied bot
    private void shootAndHelp() {
        // Try to regroup with allied that sent the message
        // While stopping and shooting at enemies in sight/radar range
        if (attackNearestEnemy()) {
            return; // Enemy detected and attacked, regrouping paused
        }

        // No enemy or bullet detected, regroup
        // Either shoot or advance
        if (shootAndAdvanceCounter % 2 == 0) {
            shootAndAdvanceCounter += 1;
            // Shoots randomly toward the target direction
            myFire(getHeading() + (Math.random() - 0.5) * Math.PI / 6, -1, false);
        } else {
            shootAndAdvanceCounter -= 1;
            headTowardCoord();
        }
    }

    // Stop and shoot at given direction for a number of steps
    private void stopAndShoot() {
        TaskAttribute currentTask = currentTasks.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            // Task completed
            currentTasks.removeFirst();
            sendLogMessage("Stop and shoot complete.");
            System.out.println("[" + myID + "] : "+ "Stop and shoot complete. calling next task : " + (currentTasks.isEmpty() ? "none" : currentTasks.getFirst().task));
            callNextTask();
        } else {
            myFire(currentTask.getShootingDirection(), currentTask.getDistanceToEnemy(), false);
            currentTask.incrementWaitingStep();
        }
    }

    // Adds firing guards to avoid hitting allies or shooting through obstacles
    public void myFire(double direction, double distanceToEnemy, boolean didIdetectEnemy) {
        // Not shooting at the beginning of the match
        if (waitBeforeShooting < WAIT_STEPS_BEFORE_SHOOTING) {
            return;
        }

        // Calculate enemy coordinates (direction is RELATIVE from radar -> convert to absolute)
        if (didIdetectEnemy) {
            double absDir = normalizeAngle(getHeading() + direction);
            double enemyX = myX + distanceToEnemy * Math.cos(absDir);
            double enemyY = myY + distanceToEnemy * Math.sin(absDir);
            // Send broadcast message when firing
            broadcast(FIGHTING_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID);
        }

//        // Checking for allies in the line of fire
//        for (IRadarResult obj : detectRadar()) {
//            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot ||
//                    obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
//
//                double d = obj.getObjectDistance();
//                double r = (obj.getObjectType() == IRadarResult.Types.TeamMainBot)
//                        ? Parameters.teamAMainBotRadius
//                        : Parameters.teamASecondaryBotRadius;
//
//                double angleToAlly = normalizeAngle(obj.getObjectDirection());
//                double angleDiff = normalizeAngle(direction - angleToAlly);
//
//                double lateral = d * Math.sin(angleDiff);   // distance perpendiculaire
//                double forward = d * Math.cos(angleDiff);   // distance devant
//
//                if (forward > 0 && Math.abs(lateral) < r + Parameters.bulletRadius + 5) {
//                    sendLogMessage("Ally in firing line (radius check). Aborting fire.");
//                    return;
//                }
//            }
//        }

        // Check if the enemy is behind an obstacle and move accordingly if so
        if (distanceToEnemy != -1) {
            for (IRadarResult obj : detectRadar()) {
                if (obj.getObjectType() == IRadarResult.Types.Wreck || obj.getObjectType() == IRadarResult.Types.TeamMainBot ||
                        obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                    double d = obj.getObjectDistance();
                    double r = obj.getObjectRadius();

                    // Both are relative to our heading
                    double angleToObj = normalizeAngle(obj.getObjectDirection());
                    double angleDiff = normalizeAngle(direction - angleToObj);

                    double lateral = d * Math.sin(angleDiff);
                    double forward = d * Math.cos(angleDiff);

                    // Obstacle blocks the line of fire if it's in front of us AND before the enemy
                    if (forward > 0 && forward < distanceToEnemy && Math.abs(lateral) < r + Parameters.bulletRadius) {
                        sendLogMessage("Enemy behind wreck: repositioning for a clearer shot.");

                        // Avoid stacking infinite reposition tasks
                        // Move to the side opposite to the obstacle's lateral position
                        double sign = (lateral >= 0) ? -1.0 : 1.0;
                        double delta = sign * (Math.PI / 6);

                        currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(50)));
                        currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(normalizeAngle(getHeading() + delta))));
                        return; // Abort fire for this step
                    }
                }
            }
        }

        fire(direction);
    }

    // Alternates shooting and advancing toward target coordinates
    private void shootAndAdvance() {
        // Try to attack nearest enemy
        if (attackNearestEnemy()) {
            return; // Enemy detected and attacked
        }
        else {
            // As there's no point in advancing toward a wall, avoid it without waiting for the counter to alternate
            // We could've made it less redundant by using myMove
            boolean wallAhead = detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
            if (wallAhead) {
                avoidWall();
                return;
            }

            // Advance and shoot
            shootAndAdvanceCounter += 1;
            if (shootAndAdvanceCounter % 2 == 0) {
                myFire(getHeading() + (Math.random() - 0.5) * Math.PI / 6, -1, false);
            } else {
                // If an obstacle is too close, myMove will handle it
                myMove();
            }
        }
    }

    // Avoid wall by turning around a bit and advancing
    public void avoidWall() {
        sendLogMessage("Avoiding wall.");
        double targetHeading = normalizeAngle(getHeading() + Math.PI + (Math.random() - 0.5) * Math.PI / 3);
        currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
        currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(targetHeading)));
    }

    // ===================== UTILS FUNCTION =====================

    private double computeDistanceTo(double x, double y) {
        double dx = x - myX;
        double dy = y - myY;
        return Math.hypot(dx, dy);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // Smallest signed angular difference (wraps around -PI/PI)
    private double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(angleDiff(dir1, dir2)) < ANGLE_PRECISION;
    }

    // If type is 0 : then look for any enemy
    // If type is 1 : then look for main bot only
    // If type is 2 : then look for secondary bot only
    private boolean isEnemyOfType(IRadarResult enemy, int type) {
        return switch (type) {
            case 0 ->
                    enemy.getObjectType() == IRadarResult.Types.OpponentMainBot || enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
            case 1 -> enemy.getObjectType() == IRadarResult.Types.OpponentMainBot;
            case 2 -> enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
            default -> false;
        };
    }

    // Returns the closest detected object from radar, or null if none found
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

    // Returns the nearest enemy of given type from the list, or null if none found
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

    // Help determining optimal turn direction to avoid obstacle (not using hysteresis)
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

    // ===================== MAIN LOOP =====================

    @Override
    public void step() {
        waitBeforeShooting++;

        // If destroyed, do nothing
        if (getHealth() <= 0) {
            return;
        }

        // Shoots at nearest enemy if any (if true then an enemy was detected and attacked)
        if (attackNearestEnemy()) {
            // Broadcasting position of enemy to allies
            IRadarResult nearestEnemy = getNearestEnemy(detectRadar(), 0);

            // Computes enemy coordinates
            double angle = nearestEnemy.getObjectDirection();
            double enemyDistance = nearestEnemy.getObjectDistance();

            double enemyX = myX + enemyDistance * Math.cos(angle);
            double enemyY = myY + enemyDistance * Math.sin(angle);
            String enemyMessage = FIGHTING_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID;
            broadcast(enemyMessage);
            return;
        }

        // Reads messages in reverse order (most recent first)
        ArrayList<String> messages = this.fetchAllMessages();
        for (int i = messages.size() - 1; i >= 0; i--) {
            if (receiveMessage(messages.get(i))) {
                return;
            }
        }

        // No enemy detected and no message received, proceed with current task
        if (!currentTasks.isEmpty()) {
            callNextTask();
            return;
        }

        sendLogMessage("No task. Holding position.");
    }

    // Checks received message and makes the bot react accordingly
    private boolean  receiveMessage(String message) {
        String[] parts = message.split(";");

        if (parts[0].equals(FIGHTING_ENEMY_MESSAGE) && parts.length == 4) {
            try {
                double enemyX = Double.parseDouble(parts[1]);
                double enemyY = Double.parseDouble(parts[2]);
                String reportingAllyID = parts[3];
                System.out.println("[" + myID + "] : "  + "Received asking for help message from : " + reportingAllyID);
                if ( computeDistanceTo(enemyX, enemyY) < Parameters.bulletRange - 100 || reportingAllyID.equals(allyAskingForHelpID) || !doCurrentTaskContain(Task.SHOOT_AND_HELP)) {
                    // The taget has been accepted, replace current tasks with the default behavior and heading toward the reported enemy
                    currentTasks.clear();
                    // Update target coordinates
                    sendLogMessage("Ally reported enemy at (" + enemyX + ", " + enemyY + "). Heading there.");
                    targetX = enemyX;
                    targetY = enemyY;
                    currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
                    currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_HELP, new TaskAttribute(reportingAllyID)));
                    callNextTask();
                    return true;
                }
            } catch (NumberFormatException e) {
                sendLogMessage("Invalid fighting enemy message format.");
            }
        }
        return false;
    }
}