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

    private static final String READY_TO_ATTACK_MESSAGE = "READY_TO_ATTACK";
    private static final String FIGHTING_ENEMY_MESSAGE = "FIGHTING_ENEMY";
    private static final String NOT_FIGHTING_ENEMY_MESSAGE = "NO_ENEMY_DETECTED";

    private int alliedReadyToAttack = 0;

    /**
     * @param attr null si pas besoin
     */
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

        public int getCurrentStep() {
            return currentStep;
        }
    }

    private enum Task {
        TURN_LEFT,
        TURN_RIGHT,
        CLOSE_DISTANCE,
        HEAD_TOWARD_ANY_ENEMY_AT_SHOOTING_RANGE,
        HEAD_TOWARD_MAIN_ENEMY_AT_SHOOTING_RANGE,
        HEAD_TOWARD_SECONDARY_ENEMY_AT_SHOOTING_RANGE,
        HEAD_TOWARD_ANY_ENEMY_AT_CLOSE_RANGE,
        HEAD_TOWARD_MAIN_ENEMY_AT_CLOSE_RANGE,
        HEAD_TOWARD_SECONDARY_ENEMY_AT_CLOSE_RANGE,
        // ajout pour mise en position (rendez-vous)
        GET_INTO_FORMATION, WAITING_FOR_ALLY_STATUS, GO_AROUND_OBJECT, TURN,
        MOVE_A_BIT, COVER_AREA, SHOOT_AND_ADVANCE, SHOOT_AND_HELP,
        // Lost recovery
        }

    private static final double TURN_HYSTERESIS = 0.02;
    private static final double SHOOTING_RANGE_OFFSET = 10.0;
    private static final double ANGLEPRECISION = 0.2;
    private final static double HEADING_PRECISION = 0.01;

    // Constantes du terrain
    private static final double ARENA_WIDTH = 3000.0;
    private static final double ARENA_HEIGHT = 2000.0;
    private static final double BOT_RADIUS = Parameters.teamAMainBotRadius;
    private static final double FRONT_SENSOR_RANGE = Parameters.teamAMainBotFrontalDetectionRange;

    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    private double targetX;
    private double targetY;

    private int shootAndAdvanceCounter = 0;

    private LinkedList<QueuedTask> currentTasks;

    @Override
    public void activate() {
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
            case MB1:
                myX = Parameters.teamAMainBot1InitX;
                myY = Parameters.teamAMainBot1InitY;
                break;
            case MB2:
                myX = Parameters.teamAMainBot2InitX;
                myY = Parameters.teamAMainBot2InitY;
                break;
            case MB3:
                myX = Parameters.teamAMainBot3InitX;
                myY = Parameters.teamAMainBot3InitY;
                break;
        }

        currentTasks = new LinkedList<>();
    }

    private void updateOdometryAfterMove() {
        myX = this.myX + Parameters.teamAMainBotSpeed * Math.cos(this.getHeading());
        myY = this.myY + Parameters.teamAMainBotSpeed * Math.sin(this.getHeading());
    }

    // ===================== END ODOMETRY CORRECTION =====================

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    // ===================== END LOST DETECTION & RECOVERY =====================

    // Smallest signed angular difference (wraps around -PI/PI)
    private double angleDiff(double a, double b) {
        double diff = a - b;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        return diff;
    }

    private boolean isSameDirection(double dir1, double dir2) {
        return Math.abs(angleDiff(dir1, dir2)) < ANGLEPRECISION;
    }

    // return true if there is a task being executed and executes it
    // Useful to chain tasks
    public void callNextTask() {
        if (!currentTasks.isEmpty()) {
            switch (currentTasks.getFirst().task) { // replaced getFirst() -> get(0)
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

    public void step() {
        System.out.println("Bot ID: " + myID + " | Position: (" + myX + ", " + myY + ") | Health: " + getHealth());
        // Correction d'odométrie à chaque step
        if (getHealth() <= 0) {
            return;
        }

        ArrayList<String> messages = this.fetchAllMessages();
        for (String message : messages) {
            if (receiveMessage(message)) {
                return;
            }
        }

        if (!currentTasks.isEmpty()) {
            callNextTask();
            return;
        }

        sendLogMessage("No task. Holding position.");
    }

    // LONGEMENT DES MURS
    private void turnLeft() {
        // Check if the turn is complete
        if (isHeadingReached(currentTasks.getFirst().attr.targetHeading)) {
            currentTasks.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.LEFT);
        }
    }

    private void turnRight() {
        // Check if the turn is complete
        if (isHeadingReached(currentTasks.getFirst().attr.targetHeading)) {
            currentTasks.removeFirst();
            callNextTask();
        } else {
            stepTurn(Parameters.Direction.RIGHT);
        }
    }

    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

    private int shootOrAdvance = 0;

    private void shootAndHelp() {
        // Try to regroup with allied that sent the message
        // While shooting at enemies
        ArrayList<IRadarResult> objects = this.detectRadar();
        for (IRadarResult object : objects) {
            if (object.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    object.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                double direction = object.getObjectDirection();
                double distance = object.getObjectDistance();
                myFire(direction, distance);
                return;
            }
        }
        // No enemy or bullet detected, regroup
        // Either shoot or advance
        if (shootOrAdvance % 2 == 0) {
            shootOrAdvance += 1;
            // Shoots randomly toward the target direction
            myFire(getHeading() + (Math.random() - 0.5) * Math.PI / 6, -1);
        } else {
            shootOrAdvance -= 1;
            headTowardCoord();
        }
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

    // stopAtShootingRange : if true, considers current bot as a shooter and stop it from advancing too much into shooting range
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
        if (nearestEnemy != null) {
            // No nearest enemy found
            broadcast(NOT_FIGHTING_ENEMY_MESSAGE + ";" + myID);
        }
        return nearestEnemy;
    }

    // --- nouvelles méthodes pour gestion RDV / déplacement en ligne droite ---

    private boolean receiveMessage(String message) {
        String[] parts = message.split(";");

        if (message.equals(READY_TO_ATTACK_MESSAGE)) {
            alliedReadyToAttack += 1;
            if (alliedReadyToAttack > 3) {
                // Heading toward enemy while shooting either randomly or targeting nearest enemy
                currentTasks.clear();
                currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
                sendLogMessage("Advancing.");
                alliedReadyToAttack = 0;
            }
            callNextTask();
            return true;
        }

        if (parts[0].equals(FIGHTING_ENEMY_MESSAGE) && parts.length == 4) {
            try {
                double enemyX = Double.parseDouble(parts[1]);
                double enemyY = Double.parseDouble(parts[2]);
                String reportingAllyID = parts[3];
                // If no enemy detected, head toward the reported coordinates
                ArrayList<IRadarResult> objects = this.detectRadar();
                IRadarResult nearestEnemy = getNearestEnemy(objects, 0);
                if (nearestEnemy == null && !doCurrentTaskContain(Task.SHOOT_AND_HELP)) {
                    sendLogMessage("Ally reported enemy at (" + enemyX + ", " + enemyY + "). Heading there.");
                    targetX = enemyX;
                    targetY = enemyY;
                    currentTasks.clear();
                    currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_HELP, new TaskAttribute(reportingAllyID)));
                    callNextTask();
                    return true;
                }
            } catch (NumberFormatException e) {
                sendLogMessage("Invalid fighting enemy message format.");
            }
        }

        if (parts[0].equals(NOT_FIGHTING_ENEMY_MESSAGE) && parts.length == 2) {
            String reportingAllyID = parts[1];
            // If currently in SHOOT_AND_HELP task targeting this ally, stop it
            if (!currentTasks.isEmpty() && currentTasks.getFirst().task == Task.SHOOT_AND_HELP) {

                TaskAttribute currentTaskAttr = currentTasks.getFirst().attr;
                if (currentTaskAttr.allyId != null && currentTaskAttr.allyId.equals(reportingAllyID)) {
                    currentTasks.clear();
                    currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
                }
            }
        }
        return false;
    }

    private void shootAndAdvance() {
        // Try to attack nearest enemy
        IRadarResult nearestEnemy = getNearestEnemy(detectRadar(), 0);
        if (attackNearestEnemy()) {
            return;
        }
        else {
            // Check if facing a wall
            boolean wallAhead = detectFront().getObjectType() == IFrontSensorResult.Types.WALL;
            if (wallAhead) {
                sendLogMessage("Wall Ahead.");
                double heading = getHeading();

                // Normaliser heading dans [0, 2π]
                while (heading < 0) heading += 2 * Math.PI;
                while (heading >= 2 * Math.PI) heading -= 2 * Math.PI;

                double targetHeading;
                if (heading < Math.PI / 2) {
                    // Face vers l'est (0), tourner vers le sud (π/2)
                    targetHeading = -Math.PI / 2;
                } else if (heading < Math.PI) {
                    // Face vers le sud (π/2), tourner vers l'ouest (π)
                    targetHeading = Math.PI/2;
                } else if (heading < 3 * Math.PI / 2) {
                    // Face vers l'ouest (π), tourner vers le nord (3π/2)
                    targetHeading = -Math.PI;
                } else {
                    // Face vers le nord (3π/2 à 2π), tourner vers l'est (0)
                    targetHeading = 0;
                }

                currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(heading + targetHeading)));
                return;
            }



            // Advance and shoot
            shootAndAdvanceCounter += 1;
            if (shootAndAdvanceCounter % 2 == 0) {
                myFire(getHeading() + (Math.random() - 0.5) * Math.PI / 6, -1);
            } else {
                myMove();
            }
        }
    }

    public void myFire(double direction, double distanceToEnemy) {
        // Calculate enemy coordinates (direction is RELATIVE from radar -> convert to absolute)
        if (distanceToEnemy != -1) {
            double absDir = normalizeAngle(getHeading() + direction);
            double enemyX = myX + distanceToEnemy * Math.cos(absDir);
            double enemyY = myY + distanceToEnemy * Math.sin(absDir);
            // Send broadcast message when firing
            broadcast(FIGHTING_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID);
        }

        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {

                double d = obj.getObjectDistance();
                double r = (obj.getObjectType() == IRadarResult.Types.TeamMainBot)
                        ? Parameters.teamAMainBotRadius
                        : Parameters.teamASecondaryBotRadius;

                double angleToAlly = normalizeAngle(obj.getObjectDirection());
                double angleDiff = normalizeAngle(direction - angleToAlly);

                double lateral = d * Math.sin(angleDiff);   // distance perpendiculaire
                double forward = d * Math.cos(angleDiff);   // distance devant

                if (forward > 0 && Math.abs(lateral) < r + Parameters.bulletRadius + 5) {
                    sendLogMessage("Ally in firing line (radius check). Aborting fire.");
                    return;
                }
            }
        }

        // Check if the enemy is behind an obstacle (wreck in the bullet path)
        if (distanceToEnemy != -1) {
            for (IRadarResult obj : detectRadar()) {
                if (obj.getObjectType() == IRadarResult.Types.Wreck) {
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
                        if (!doCurrentTaskContain(Task.TURN) && !doCurrentTaskContain(Task.MOVE_A_BIT) && !doCurrentTaskContain(Task.GO_AROUND_OBJECT)) {
                            // Move to the side opposite to the obstacle's lateral position
                            double sign = (lateral >= 0) ? -1.0 : 1.0;
                            double delta = sign * (Math.PI / 6);

                            currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(50)));
                            currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(normalizeAngle(getHeading() + delta))));
                        }
                        return; // Abort fire for this step
                    }
                }
            }
        }

        fire(direction);
    }

    // Déplacement en ligne droite vers (targetX, targetY)
    private void headTowardCoord() {
        double dx = targetX - myX;
        double dy = targetY - myY;
        double distance = Math.hypot(dx, dy);

        if (distance <= 5.0) {

            sendLogMessage("Reached target coordinates (" + targetX + ", " + targetY + ").");
            // Remove SHOOT_AND_HELP task and switch to SHOOT_AND_ADVANCE
            if (currentTasks.getFirst().task == Task.SHOOT_AND_HELP) {
                currentTasks.removeFirst();
                currentTasks.addFirst(new QueuedTask(Task.SHOOT_AND_ADVANCE));
                callNextTask();
            }
            return;
        }

        double directionToTarget = Math.atan2(dy, dx);
        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(directionToTarget);
        if (turnDirection != null) {
            stepTurn(turnDirection);
        } else {
            myMove();
        }
    }

    // Déplacement

    public void myMove() {
        // Check if facing wall
        if (detectFront().getObjectType() == IFrontSensorResult.Types.WALL) {
            sendLogMessage("Wall detected ahead: initiating avoidance maneuver.");
            // Turn away from the wall
            double targetHeading = normalizeAngle(getHeading() + Math.random() * Math.PI); // Turn right 90 degrees
            currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(10)));
            currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(targetHeading)));
            return;
        }

        if (currentTasks.getFirst().task.equals(Task.GO_AROUND_OBJECT)) {
            goAroundObject();
            return;
        }

        if (currentTasks.getFirst().task.equals(Task.MOVE_A_BIT)) {
            updateOdometryAfterMove();
            move();
            return;
        }

        if (isObstacleTooClose(1.04)) {
            // Check if we are alreadyTrying to go around an object
            if (!doCurrentTaskContain(Task.GO_AROUND_OBJECT)) {
                currentTasks.addFirst(new QueuedTask(Task.GO_AROUND_OBJECT));
                sendLogMessage("Object too close: initiating go around maneuver.");
            } else {
                sendLogMessage("Object too close: already trying to go around.");
            }
            return;
        }

        updateOdometryAfterMove();
        move();
    }

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

    private void goAroundObject() {
        // remove GO_AROUND_OBJECT task
        currentTasks.removeFirst();

        // Determine nearest object position relative to us
        IRadarResult closestObject = getRadarClosestObject();
        if (closestObject == null) {
            sendLogMessage("No object detected: stopping go around maneuver.");
            return;
        }

        // 1 chance out of 30 to just move a back a bit instead of going around
        if (Math.random() < 5.0 / 20.0) {
            sendLogMessage("Go around maneuver: opting to move back a bit instead.");
            // Turn around first
            currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(normalizeAngle(getHeading() + Math.PI))));
            currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(40)));
        }
        else {
            // objectDirection is RELATIVE to our current heading
            double targetHeading = getTargetHeading(closestObject);

            // Sequence: turn away, then move forward a bit to clear the obstacle
            currentTasks.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(40)));
            currentTasks.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(targetHeading)));
        }
    }

    private double getTargetHeading(IRadarResult closestObject) {
        double objectDirection = normalizeAngle(closestObject.getObjectDirection());

        // Turn AWAY from the obstacle (if obstacle is on the right, turn left; if on the left, turn right)
        double sideStep = (objectDirection >= 0) ? -Math.PI / 2 : Math.PI / 2;

        // Small extra bias so we don't end up tangentially glued to the wreck
        double bias = (objectDirection >= 0) ? -Math.PI / 12 : Math.PI / 12;

        double targetHeading = normalizeAngle(getHeading() + sideStep + bias);
        return targetHeading;
    }

    private boolean doCurrentTaskContain(Task obj) {
        for (QueuedTask task : currentTasks) {
            if (task.task == obj) {
                return true;
            }
        }
        return false;
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

    private void moveAbit() {
        TaskAttribute currentTask = currentTasks.getFirst().attr;
        if (currentTask.isTargetWaitingStepsReached()) {
            // Move complete
            currentTasks.removeFirst();
            sendLogMessage("Move a bit complete.");
            callNextTask();
        } else {
            // If we're still blocked, change heading a bit and retry instead of ramming the wreck forever
            IFrontSensorResult front = detectFront();
            if (front.getObjectType() == IFrontSensorResult.Types.Wreck || front.getObjectType() == IFrontSensorResult.Types.WALL) {
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

    private boolean attackNearestEnemy() {
        ArrayList<IRadarResult> objects = this.detectRadar();
        IRadarResult nearestEnemy = getNearestEnemy(objects, 0);
        if (nearestEnemy != null) {
            // Suppose there's no ally in front of us
            double direction = nearestEnemy.getObjectDirection();
            double distance = nearestEnemy.getObjectDistance();
            myFire(direction, distance);
            return true;
        }
        return false;
    }
}