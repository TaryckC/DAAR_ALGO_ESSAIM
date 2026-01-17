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

    private  static final String MSG_ENEMY_FOUND = "ENEMY_MARKED";
    private static final String READY_TO_ATTACK_MESSAGE = "READY_TO_ATTACK";
    private static final String FIGHTINHG_ENEMY_MESSAGE = "FIGHTING_ENEMY";
    private static final String NOT_FIGHTING_ENEMY_MESSAGE = "NO_ENEMY_DETECTED";

    private int alliedReadyToAttack = 0;

    private static class TaskAttribute {
        double targetHeading;
        double targetX;
        double targetY;
        final int TARGET_STEPS = 100;
        private int currentStep;
        private int targetWaitingSteps;

        // nouvelle variable pour stocker la réponse de l'allié (true => il bouge, false => il est à l'arrêt, null => pas de réponse)
        private Boolean allyIsMovingResponse = null;

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

        public int incrementWaitingStep() {
            this.currentStep++;
            return currentStep;
        }

        TaskAttribute(double targetX, double targetY) {
            this.targetX = targetX;
            this.targetY = targetY;
        }

        TaskAttribute() {
            currentStep = 0;
        }

        // return pre-increment value
        public int incrementStep() {
            return currentStep++;
        }

        public int getCurrentStep() {
            return currentStep;
        }

        public boolean isTargetStepsReached() {
            return currentStep >= TARGET_STEPS;
        }

        // --- nouvelles méthodes pour la communication ---
        public void setAllyResponse(Boolean moving) {
            this.allyIsMovingResponse = moving;
        }

        public Boolean getAllyResponse() {
            return this.allyIsMovingResponse;
        }

        // marque l'attente comme terminée (permettre suppression immédiate de la tâche WAITING)
        public void markWaitingComplete() {
            this.currentStep = this.targetWaitingSteps;
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
        MOVE_A_BIT, COVER_AREA, SHOOT_AND_ADVANCE, SHOOT_AND_HELP;
    }

    private static final double TURN_HYSTERESIS = 0.02;
    private static final double SHOOTING_RANGE_OFFSET = 10.0;
    private static final double ANGLEPRECISION = 0.2;
    private final static double HEADING_PRECISION = 0.01;

    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    // --- nouveaux champs pour RDV / formation ---
    private boolean isOccupied = false;
    private double rendezvousX;
    private double rendezvousY;
    private double rendezvousAngle;
    private double targetX;
    private double targetY;
    private double RDV_OFFSET_X = -70.0;
    private double RDV_OFFSET_Y = 100.0;
    private String RDV_MESSAGE = "RENDEZVOUS";

    private int shootAndAdvanceCounter = 0;

    private LinkedList<Task> currentTasks;
    private LinkedList<TaskAttribute> currentTaskAttributes;

    @Override
    public void activate() {
        // If bot detects allied main bot, downward and upward its position -> he's determined to be MB2
        // If there's only an allied mainBot upward, then it's MB1
        // Else it's MB3
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
        currentTaskAttributes = new LinkedList<>();
    }

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
    public boolean callNextTask() {
        if (!currentTasks.isEmpty()) {
            switch (currentTasks.get(0)) { // replaced getFirst() -> get(0)
                // TODO : Find a way to merge these similar tasks
                case SHOOT_AND_HELP :
                    shootAndHelp();
                    return true;
                case SHOOT_AND_ADVANCE:
                    shootAndAdvance();
                    return true;
                case COVER_AREA:
                    coverArea();
                    return true;
                case GO_AROUND_OBJECT:
                    goAroundObject();
                    return true;
                case MOVE_A_BIT :
                    moveAbit();
                    return true;
                case TURN :
                    turn();
                    return true;
                case HEAD_TOWARD_ANY_ENEMY_AT_SHOOTING_RANGE:
                    headTowardNearestEnemy(true, 0);
                    return true;
                case HEAD_TOWARD_MAIN_ENEMY_AT_SHOOTING_RANGE:
                    headTowardNearestEnemy(true, 1);
                    return true;
                case HEAD_TOWARD_SECONDARY_ENEMY_AT_SHOOTING_RANGE:
                    headTowardNearestEnemy(true, 2);
                    return true;
                case HEAD_TOWARD_ANY_ENEMY_AT_CLOSE_RANGE:
                    headTowardNearestEnemy(false, 0);
                    return true;
                case HEAD_TOWARD_MAIN_ENEMY_AT_CLOSE_RANGE:
                    headTowardNearestEnemy(false, 1);
                    return true;
                case HEAD_TOWARD_SECONDARY_ENEMY_AT_CLOSE_RANGE:
                    headTowardNearestEnemy(false, 2);
                    return true;
                case GET_INTO_FORMATION:
                    return getIntoFormation();
                case TURN_LEFT:
                    turnLeft();
                    return true;
                case TURN_RIGHT:
                    turnRight();
                    return true;
                case CLOSE_DISTANCE:
                    closeDistance();
                    return true;
            }
        }
        return false;
    }

    public void step() {
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

        // comportement par défaut : rester en attente (main bots)
        sendLogMessage("No task. Holding position.");
    }

    // LONGEMENT DES MURS
    private boolean turnLeft() {
        if (currentTaskAttributes.isEmpty()) {
            // targetHeading = getHeading() - Math.PI / 2;
            this.currentTaskAttributes.addFirst(new TaskAttribute(getHeading() - 0.5 * Math.PI));
            sendLogMessage("Turning left.");
        }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttributes.getFirst().targetHeading)) {
            currentTasks.remove(0); // removed removeFirst()
            currentTaskAttributes.removeFirst();
            sendLogMessage("Turn complete.");
            callNextTask();
            return false;
        } else {
            stepTurn(Parameters.Direction.LEFT);
            return true;
        }
    }

    private boolean turnRight() {
        if (currentTaskAttributes.isEmpty()) {
            // targetHeading = getHeading() - Math.PI / 2;
            this.currentTaskAttributes.addFirst(new TaskAttribute(getHeading() + 0.5 * Math.PI));
            sendLogMessage("Turning right.");
        }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttributes.getFirst().targetHeading)) {
            currentTasks.remove(0); // removed removeFirst()
            currentTaskAttributes.removeFirst();
            sendLogMessage("Turn complete.");
            callNextTask();
            return false;
        } else {
            stepTurn(Parameters.Direction.RIGHT);
            return true;
        }
    }

    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

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
        headTowardCoord();
    }

    private boolean closeDistance() {
        if (currentTaskAttributes.isEmpty()) {
            this.currentTaskAttributes.add(new TaskAttribute());
        }
        // Check if the move is complete
        if (currentTaskAttributes.getFirst().isTargetStepsReached()) {
            currentTasks.removeFirst();
            currentTaskAttributes.removeFirst();
            callNextTask();
            return false;
        } else {
            myMove();
            currentTaskAttributes.getFirst().incrementStep();
            return true;
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
        switch (type) {
            case 0:
                return enemy.getObjectType() == IRadarResult.Types.OpponentMainBot || enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
            case 1:
                return enemy.getObjectType() == IRadarResult.Types.OpponentMainBot;
            case 2:
                return enemy.getObjectType() == IRadarResult.Types.OpponentSecondaryBot;
        }
        return false;
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

    // NOTE : Radar has a limited range.
    // Currently follows the direct path toward enemy, without checking for obstacles
    private boolean headTowardNearestEnemy(boolean stopAtShootingRange, int type) {
        // No attributes needed for this task
        ArrayList<IRadarResult> objects = this.detectRadar();
        IRadarResult nearestEnemy = getNearestEnemy(objects, type);
        if (nearestEnemy != null) {
            this.sendLogMessage("Heading toward nearest enemy of type " + type + " at distance " + nearestEnemy.getObjectDistance());
            double enemyDirection = nearestEnemy.getObjectDirection();
            // Determine optimal turn direction
            Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(enemyDirection);
            if (turnDirection != null) {
                // Need to turn
                stepTurn(turnDirection);
                return true;
            } else {
                // Facing enemy
                if (stopAtShootingRange) {
                    double distanceToEnemy = nearestEnemy.getObjectDistance();
                    double shootingRange = Parameters.bulletRange - SHOOTING_RANGE_OFFSET;
                    if (distanceToEnemy > shootingRange) {
                        myMove(); // Advance toward enemy
                    } else {
                        sendLogMessage("In shooting range of enemy. Holding position.");
                    }
                } else {
                    // No need to stop, just go ahead
                    myMove();
                }
                return true;
            }
        }

        this.sendLogMessage("No enemy of type " + type + " detected.");

        // No action taken
        // Engage call next task
        if (!currentTasks.isEmpty()) currentTasks.remove(0); // removed removeFirst()
        callNextTask();
        return false;
    }

    // --- nouvelles méthodes pour gestion RDV / déplacement en ligne droite ---

    private boolean receiveMessage(String message) {
        String[] parts = message.split(";");
        if (parts[0].equals(RDV_MESSAGE) && parts.length == 4) {
            try {
                rendezvousX = Double.parseDouble(parts[1]);
                rendezvousY = Double.parseDouble(parts[2]);
                rendezvousAngle = Double.parseDouble(parts[3]);
                isOccupied = true;
                currentTasks.clear();
                currentTasks.add(0, Task.GET_INTO_FORMATION);
                sendLogMessage("Received rendez-vous point at (" + rendezvousX + ", " + rendezvousY + ").");
                return true;
            } catch (NumberFormatException e) {
                sendLogMessage("Invalid rendez-vous message format.");
            }
        }
        if (message.equals(READY_TO_ATTACK_MESSAGE)) {
            alliedReadyToAttack += 1;
            if (alliedReadyToAttack > 3) {
                // Heading toward enemy while shooting either randomly or targeting nearest enemy
                currentTasks.clear();
                currentTasks.addFirst(Task.SHOOT_AND_ADVANCE);
                sendLogMessage("Advancing.");
                alliedReadyToAttack = 0;
            }
            callNextTask();
            return true;
        }

        if (parts[0].equals(FIGHTINHG_ENEMY_MESSAGE) && parts.length == 4) {
            try {
                double enemyX = Double.parseDouble(parts[1]);
                double enemyY = Double.parseDouble(parts[2]);
                String reportingAllyID = parts[3];
                // If no enemy detected, head toward the reported coordinates
                ArrayList<IRadarResult> objects = this.detectRadar();
                IRadarResult nearestEnemy = getNearestEnemy(objects, 0);
                if (nearestEnemy == null) {
                    sendLogMessage("Ally reported enemy at (" + enemyX + ", " + enemyY + "). Heading there.");
                    if (currentTasks.getFirst() == Task.SHOOT_AND_HELP) {
                        callNextTask();
                    }
                    targetX = enemyX;
                    targetY = enemyY;
                    currentTasks.clear();
                    currentTaskAttributes.clear();
                    currentTasks.addFirst(Task.SHOOT_AND_HELP);
                    currentTaskAttributes.addFirst(new TaskAttribute(reportingAllyID));
                    return true;
                }
            } catch (NumberFormatException e) {
                sendLogMessage("Invalid fighting enemy message format.");
            }
        }

        if (parts[0] == NOT_FIGHTING_ENEMY_MESSAGE && parts.length == 2) {
            String reportingAllyID = parts[1];
            // If currently in SHOOT_AND_HELP task targeting this ally, stop it
            if (currentTasks.getFirst() == Task.SHOOT_AND_HELP) {
                TaskAttribute currentTaskAttr = currentTaskAttributes.getFirst();
                if (currentTaskAttr.allyId.equals(reportingAllyID)) {
                    currentTaskAttributes.clear();
                    currentTasks.clear();
                    currentTasks.addFirst(Task.SHOOT_AND_ADVANCE);
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
                    System.out.println("Facing East, turning South");
                } else if (heading < Math.PI) {
                    // Face vers le sud (π/2), tourner vers l'ouest (π)
                    targetHeading = Math.PI/2;
                    System.out.println("Facing South, turning West");
                } else if (heading < 3 * Math.PI / 2) {
                    // Face vers l'ouest (π), tourner vers le nord (3π/2)
                    targetHeading = -Math.PI;
                    System.out.println("Facing West, turning North");
                } else {
                    // Face vers le nord (3π/2 à 2π), tourner vers l'est (0)
                    targetHeading = 0;
                    System.out.println("Facing North, turning East");
                }

                System.out.println("Heading (normalized): " + heading + " -> Target: " + targetHeading);
                currentTasks.addFirst(Task.TURN);
                currentTaskAttributes.addFirst(new TaskAttribute(targetHeading));
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
        // Calculate enemy coordinates
        if (distanceToEnemy != -1) {
            double ennemyX = myX + distanceToEnemy * Math.cos(direction);
            double ennemyY = myY + distanceToEnemy * Math.sin(direction);
            // Send broadcast message when firing
            broadcast(FIGHTINHG_ENEMY_MESSAGE + ";" + ennemyX + ";" + ennemyY + ";" + myID);
        }

        // Check if about to fire toward an ally
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                double allyDirection = obj.getObjectDirection();
                double angleDifference = angleDiff(direction, allyDirection);
                if (Math.abs(angleDifference) < ANGLEPRECISION) {
                    sendLogMessage("Ally in firing line. Aborting fire.");
                    return; // Abort firing
                }
            }
        }

        fire(direction);
    }

    // Trouve l'index du coin le plus proche
    // 0: Top-left, 1: Top-right, 2: Bottom-left, 3: Bottom-right
    private int closestCornerIndex(double x, double y) {
        double[][] corners = {
                {0.0, 0.0},
                {3000, 0.0},
                {0.0, 2000},
                {3000, 2000}
        };
        int closestIndex = -1;
        double minDistance = Double.MAX_VALUE;
        for (int i = 0; i < corners.length; i++) {
            double dx = x - corners[i][0];
            double dy = y - corners[i][1];
            double distance = Math.hypot(dx, dy);
            if (distance < minDistance) {
                minDistance = distance;
                closestIndex = i;
            }
        }
        return closestIndex;
    }

    // Déplacement en ligne droite vers (targetX, targetY)
    private void headTowardCoord() {
        double dx = targetX - myX;
        double dy = targetY - myY;
        double distance = Math.hypot(dx, dy);

        if (distance <= 5.0) {
            sendLogMessage("Reached target coordinates (" + targetX + ", " + targetY + ").");
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

    private boolean getIntoFormation() {
        // définir position cible selon myID
        if (!(Math.abs(myX - targetX) < 5.0 && Math.abs(myY - targetY) < 5.0)) {
            if (myID.equals(MB1)) {
                targetX = rendezvousX - RDV_OFFSET_X;
                targetY = rendezvousY - (RDV_OFFSET_Y + 35);
            } else if (myID.equals(MB2)) {
                targetX = rendezvousX - (RDV_OFFSET_X + 50);
                targetY = rendezvousY - 20;
            } else {
                targetX = rendezvousX - RDV_OFFSET_X;
                targetY = rendezvousY + RDV_OFFSET_Y;
            }

            sendLogMessage("Heading to formation (" + targetX + ", " + targetY + ")");
            headTowardCoord();
        }
        else if (!isSameDirection(getHeading(), rendezvousAngle)) {
            // Aim at rendezvousAngle
            double directionToTarget = rendezvousAngle;
            Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(directionToTarget);
            if (turnDirection != null) {
                stepTurn(turnDirection);
                sendLogMessage("Adjusting heading to rendezvous angle.");
            }
        }
        else {
                sendLogMessage("Main bot in formation position.");
                broadcast(READY_TO_ATTACK_MESSAGE);
                isOccupied = false;
                if (!currentTasks.isEmpty())
                    currentTasks.removeFirst();
                currentTasks.addFirst(Task.COVER_AREA);
                alliedReadyToAttack+=1;
                callNextTask();
                return false;
            }
            return true;
    }

    // Déplacement

    public void myMove() {
        // Check for WAITING_FOR_ALLY_STATUS task
        for (int i = 0; i < currentTasks.size(); i++) {
            if (currentTasks.get(i) == Task.WAITING_FOR_ALLY_STATUS) {
                if (currentTaskAttributes.get(i).isTargetWaitingStepsReached()) {
                    // Remove the waiting task
                    currentTasks.remove(i);
                    currentTaskAttributes.remove(i);
                    break;
                } else {
                    currentTaskAttributes.get(i).incrementWaitingStep();
                    sendLogMessage("Waiting for ally status... Step " + currentTaskAttributes.get(i).getCurrentStep());
                    return; // Do not move while waiting
                }
            }
        }

        if (currentTasks.getFirst().equals(Task.GO_AROUND_OBJECT)) {
            goAroundObject();
            return;
        }

        if (currentTasks.getFirst().equals(Task.MOVE_A_BIT)) {
                myX += Parameters.teamAMainBotSpeed * Math.cos(getHeading());
                myY += Parameters.teamAMainBotSpeed * Math.sin(getHeading());
                move();
                return;
        }

        if (isObstacleTooClose(1.04)) {
            // Check if we are alreadyTrying to go around an object
            if (!doCurrentTaskContain(Task.GO_AROUND_OBJECT)) {
                currentTasks.addFirst(Task.GO_AROUND_OBJECT);
                sendLogMessage("Ally too close: initiating go around maneuver.");
            } else {
                System.out.println("couou");
                sendLogMessage("Ally too close: already trying to go around.");
            }
            return;
        }

        myX+=Parameters.teamAMainBotSpeed*Math.cos(getHeading());
        myY+=Parameters.teamAMainBotSpeed*Math.sin(getHeading());
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
        // Determine nearest object position relative to us
        IRadarResult closestObject = getRadarClosestObject();
        if (closestObject == null) {
            // No object detected, remove the task
            int index = currentTasks.indexOf(Task.GO_AROUND_OBJECT);
            if (index != -1) {
                currentTasks.remove(index);
            }
            sendLogMessage("No object detected: stopping go around maneuver.");
            return;
        }

        if (!currentTaskAttributes.contains(Task.GO_AROUND_OBJECT)){
            double objectDirection = closestObject.getObjectDirection();
            // Determine turn direction to go around (always turn right for simplicity)
            double turnDirection = objectDirection + Math.PI / 2;
            // Add turnaround Task
            // remove existing GO_AROUND_OBJECT task to avoid duplication
            int index = currentTasks.indexOf(Task.GO_AROUND_OBJECT);
            if (index != -1) {
                currentTasks.remove(index);
            }
            currentTasks.addFirst(Task.MOVE_A_BIT);
            currentTaskAttributes.addFirst(new TaskAttribute(20)); // Move a bit forward
            currentTasks.addFirst(Task.TURN);
            currentTaskAttributes.addFirst(new TaskAttribute(turnDirection));
        }
    }

    private boolean doCurrentTaskContain(Task task) {
        for (Task t : currentTasks) {
            if (t == task) {
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
        TaskAttribute currentTask = currentTaskAttributes.getFirst();
        double targetHeading = currentTask.targetHeading;
        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(targetHeading);
        if (turnDirection != null) {
            stepTurn(turnDirection);
        } else {
            // Turn complete
            currentTasks.removeFirst();
            currentTaskAttributes.removeFirst();
            sendLogMessage("Turn complete.");
            callNextTask();
        }
    }

    private void moveAbit() {
        TaskAttribute currentTask = currentTaskAttributes.getFirst();
        if (currentTask.isTargetWaitingStepsReached()) {
            // Move complete
            currentTasks.removeFirst();
            currentTaskAttributes.removeFirst();
            sendLogMessage("Move a bit complete.");
            callNextTask();
        } else {
            myMove();
            currentTask.incrementWaitingStep();
        }
    }

    private boolean attackNearestEnemy() {
        ArrayList<IRadarResult> objects = this.detectRadar();
        IRadarResult nearestEnemy = getNearestEnemy(objects, 0);
        if (nearestEnemy != null) {
            // Suppoe there's no ally in front of us
            double direction = nearestEnemy.getObjectDirection();
            double distance = nearestEnemy.getObjectDistance();
            myFire(direction, distance);
            System.out.println("sent fire to direction " + direction + " at distance " + distance);
            return true;
        }
        return false;
    }

    // While waiting for ally to join the formation, cover the area by spraying bullets
    private void coverArea() {
        // Try to attack nearest enemy
        if (!attackNearestEnemy()) {
            // Spray randomly
            myFire(getHeading() + (Math.random() - 0.5) * Math.PI / 4, -1);
        }
    }

    public boolean arrivedAtCoord(double x, double y) {
        double dx = myX - x;
        double dy = myY - y;
        double distance = Math.hypot(dx, dy);
        return distance < 5.0;
    }
}
