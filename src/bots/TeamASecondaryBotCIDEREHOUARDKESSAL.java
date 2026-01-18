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

    private static class QueuedTask {
        final Task task;
        final TaskAttribute attr; // null si pas besoin

        QueuedTask(Task task) {
            this.task = task;
            this.attr = null;
        }
        QueuedTask(Task task, TaskAttribute attr) {
            this.task = task;
            this.attr = attr;
        }
    }

    private static final String SB1 = "Secondary-bot-1";
    private static final String SB2 = "Secondary-bot-2";

    private static final String WHOAREYOU = "WHOAREYOU";
    private static final String IAM = "IAM";

    // Messages pour la localisation
    private static final String REQUEST_POSITION = "REQUEST_POS";  // Demande de position: REQUEST_POS;myID
    private static final String POSITION_RESPONSE = "POS_RESP";    // Réponse: POS_RESP;targetID;myX;myY;myID

    private enum Task {
        // MOVEMENT TASKS
        TURN_LEFT,
        TURN_RIGHT,
        CLOSE_DISTANCE,

        // SCOUTING TASKS
        SCOUTING_TASK,

        // FORMATION TASKS
        GET_INTO_FORMATION,

        // MESSAGE TASKS
        ENEMY_DETECTED_AND_MARKED,

        // ATTACK TASKS
        ATTACK_NEAREST_ENEMY, MOVE_FORWARD, TURN_TOWARD_TARGET, WAITING_FOR_ALLY_STATUS,

        // LOST RECOVERY
        LOST_WANDERING, ROAM_AND_AVOID_ATTACKS, MOVE_A_BIT, TURN // Robot perdu, se déplace aléatoirement pour trouver un allié
    }

    private static class TaskAttribute {
        double targetHeading;
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

        public boolean isTargetWaitingStepsReached() {
            return currentStep >= targetWaitingSteps;
        }

        public int incrementWaitingStep() {
            this.currentStep++;
            return currentStep;
        }

        public int getCurrentStep() {
            return currentStep;
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

    private static final double TURN_HYSTERESIS = 0.05;
    private static final double SHOOTING_RANGE_OFFSET = 10.0;
    private static final double ANGLEPRECISION = 0.01;
    private final static double HEADING_PRECISION = 0.01;

    // Constantes du terrain
    private static final double ARENA_WIDTH = 3000.0;
    private static final double ARENA_HEIGHT = 2000.0;
    private static final double BOT_RADIUS = Parameters.teamASecondaryBotRadius;
    private static final double FRONT_SENSOR_RANGE = Parameters.teamASecondaryBotFrontalDetectionRange;

    private LinkedList<QueuedTask> taskQueue;

    private boolean accpetingNewMessages;
    private boolean isMoving;

    // ODOMETRY
    private String myID;
    private double myX;
    private double myY;

    // Lost detection
    private int stepsSinceLastCorrection = 0;      // Nombre de steps depuis la dernière correction
    private static final int LOST_THRESHOLD = 2000; // Seuil pour considérer le robot comme perdu
    private boolean isLost = false;                 // État perdu
    private double wanderDirection = 0;             // Direction de déplacement aléatoire
    private int wanderSteps = 0;                    // Compteur de steps dans la direction actuelle

    private static final String READY_TO_ATTACK_MESSAGE = "READY_TO_ATTACK";
    private int alliedReadyToAttackCount = 0;


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
        accpetingNewMessages = true;
        isMoving = true;

        taskQueue.addFirst(new QueuedTask(Task.SCOUTING_TASK));

    }

    // ===================== ODOMETRY CORRECTION =====================

    /**
     * Corrige l'odométrie en utilisant la détection des murs.
     * Réinitialise le compteur de perte si correction appliquée.
     */
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
        boolean corrected = false;

        if (Math.abs(heading - 0) < tolerance || Math.abs(heading - 2*Math.PI) < tolerance) {
            double correctedX = ARENA_WIDTH - distanceToWall - BOT_RADIUS;
            if (Math.abs(myX - correctedX) > 50) {
                System.out.println("[ODOMETRY " + myID + "] Wall EAST correction: myX " + myX + " -> " + correctedX);
                myX = correctedX;
                corrected = true;
            }
        }
        else if (Math.abs(heading - Math.PI/2) < tolerance) {
            double correctedY = ARENA_HEIGHT - distanceToWall - BOT_RADIUS;
            if (Math.abs(myY - correctedY) > 50) {
                System.out.println("[ODOMETRY " + myID + "] Wall SOUTH correction: myY " + myY + " -> " + correctedY);
                myY = correctedY;
                corrected = true;
            }
        }
        else if (Math.abs(heading - Math.PI) < tolerance) {
            double correctedX = distanceToWall + BOT_RADIUS;
            if (Math.abs(myX - correctedX) > 50) {
                System.out.println("[ODOMETRY " + myID + "] Wall WEST correction: myX " + myX + " -> " + correctedX);
                myX = correctedX;
                corrected = true;
            }
        }
        else if (Math.abs(heading - 3*Math.PI/2) < tolerance || Math.abs(heading + Math.PI/2) < tolerance) {
            double correctedY = distanceToWall + BOT_RADIUS;
            if (Math.abs(myY - correctedY) > 50) {
                System.out.println("[ODOMETRY " + myID + "] Wall NORTH correction: myY " + myY + " -> " + correctedY);
                myY = correctedY;
                corrected = true;
            }
        }

        if (corrected) {
            resetLostState();
        }
    }

    /**
     * Empêche les positions impossibles (en dehors du terrain).
     */
    private void clampPositionToArena() {
        double margin = BOT_RADIUS;
        double oldX = myX, oldY = myY;

        if (myX < margin) myX = margin;
        if (myX > ARENA_WIDTH - margin) myX = ARENA_WIDTH - margin;
        if (myY < margin) myY = margin;
        if (myY > ARENA_HEIGHT - margin) myY = ARENA_HEIGHT - margin;

        if (oldX != myX || oldY != myY) {
            System.out.println("[ODOMETRY " + myID + "] Clamped position from (" + oldX + ", " + oldY + ") to (" + myX + ", " + myY + ")");
        }
    }

    /**
     * Met à jour l'odométrie après un mouvement.
     */
    private void updateOdometryAfterMove() {
        myX += Parameters.teamASecondaryBotSpeed * Math.cos(getHeading());
        myY += Parameters.teamASecondaryBotSpeed * Math.sin(getHeading());
        clampPositionToArena();
        stepsSinceLastCorrection++; // Incrémenter le compteur d'incertitude
    }

    // ===================== END ODOMETRY CORRECTION =====================

    // ===================== LOST DETECTION & RECOVERY =====================

    /**
     * Réinitialise l'état perdu après une correction d'odométrie.
     */
    private void resetLostState() {
        stepsSinceLastCorrection = 0;
        if (isLost) {
            System.out.println("[LOST " + myID + "] Position recovered! No longer lost.");
            isLost = false;
            // Retirer la tâche LOST_WANDERING si présente
            if (!taskQueue.isEmpty() && taskQueue.getFirst().task == Task.LOST_WANDERING) {
                taskQueue.removeFirst();
            }
            // Ajouter une tâche par défaut si la liste est vide
            if (taskQueue.isEmpty()) {
                taskQueue.addFirst(new QueuedTask(Task.SCOUTING_TASK));
                System.out.println("[LOST " + myID + "] Resuming with SCOUTING_TASK.");
            }
        }
    }

    /**
     * Vérifie si le robot est perdu (trop de steps sans correction d'odométrie).
     */
    private void checkIfLost() {
        if (!isLost && stepsSinceLastCorrection > LOST_THRESHOLD) {
            System.out.println("[LOST " + myID + "] Robot is LOST! Steps without correction: " + stepsSinceLastCorrection);
            isLost = true;
            // Ajouter la tâche de déplacement aléatoire
            taskQueue.clear();
            taskQueue.addFirst(new QueuedTask(Task.LOST_WANDERING));
            // Choisir une direction aléatoire initiale
            wanderDirection = Math.random() * 2 * Math.PI;
            wanderSteps = 0;
        }
    }

    /**
     * Comportement quand le robot est perdu : se déplacer aléatoirement
     * et chercher un allié pour recaler l'odométrie.
     */
    private void lostWanderingBehavior() {
        sendLogMessage("LOST! Wandering to find ally...");

        // Chercher un allié sur le radar
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.TeamMainBot ||
                result.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                // Allié trouvé ! Demander sa position
                System.out.println("[LOST " + myID + "] Ally detected! Requesting position...");
                broadcast(REQUEST_POSITION + ";" + myID);
                return; // Attendre la réponse
            }
        }

        // Pas d'allié détecté, continuer à errer
        // Changer de direction si obstacle ou tous les X steps
        IFrontSensorResult front = detectFront();
        wanderSteps++;

        if (front.getObjectType() == IFrontSensorResult.Types.WALL ||
            front.getObjectType() == IFrontSensorResult.Types.Wreck ||
            wanderSteps > 200) {
            // Choisir une nouvelle direction aléatoire
            wanderDirection = Math.random() * 2 * Math.PI;
            wanderSteps = 0;
            System.out.println("[LOST " + myID + "] Changing wander direction to: " + wanderDirection);
        }

        // Se tourner vers la direction souhaitée et avancer
        double angleDiff = normalizeAngle(wanderDirection - getHeading());
        if (Math.abs(angleDiff) > 0.1) {
            if (angleDiff > 0) {
                stepTurn(Parameters.Direction.RIGHT);
            } else {
                stepTurn(Parameters.Direction.LEFT);
            }
        } else {
            // Direction OK, avancer
            if (front.getObjectType() == IFrontSensorResult.Types.NOTHING) {
                updateOdometryAfterMove();
                move();
            }
        }
    }

    /**
     * Normalise un angle dans [-PI, PI]
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    /**
     * Recale l'odométrie à partir de la position d'un allié détecté.
     * @param allyX Position X de l'allié
     * @param allyY Position Y de l'allié
     * @param allyDirection Direction de l'allié par rapport à nous (radar)
     * @param allyDistance Distance de l'allié (radar)
     */
    private void recalibrateFromAlly(double allyX, double allyY, double allyDirection, double allyDistance) {
        // Notre position = position allié - vecteur vers nous
        double newX = allyX - allyDistance * Math.cos(allyDirection);
        double newY = allyY - allyDistance * Math.sin(allyDirection);

        System.out.println("[LOST " + myID + "] Recalibrating from ally position (" + allyX + ", " + allyY + ")");
        System.out.println("[LOST " + myID + "] Old position: (" + myX + ", " + myY + ") -> New position: (" + newX + ", " + newY + ")");

        myX = newX;
        myY = newY;
        clampPositionToArena();
        resetLostState();
    }

    // ===================== END LOST DETECTION & RECOVERY =====================

    // return true if there is a task being executed and executes it
    // Useful to chain tasks
    public boolean callNextTask() {
        if (!taskQueue.isEmpty()) {
            switch (taskQueue.getFirst().task) {
                case MOVE_A_BIT:
                    moveAbit();
                    return true;
                case TURN :
                    turn();
                    return true;
                case LOST_WANDERING:
                    lostWanderingBehavior();
                    return true;
                case MOVE_FORWARD:
                    //sendLogMessage("Moving forward.");
                    return moveForward();
                case SCOUTING_TASK:
                    sendLogMessage("Scouting...");
                    scoutBehavior();
                    return true;
                case GET_INTO_FORMATION:
                    return getIntoFormation();
                case TURN_LEFT:
                    //sendLogMessage("Turning left");
                    turnLeft();
                    return true;
                case TURN_RIGHT:
                    //sendLogMessage("Turning right");
                    turnRight();
                    return true;
                case ROAM_AND_AVOID_ATTACKS:
                    roamAndAvoidAttacksBehavior();
                    return true;
            }
        }
        return false;
    }

    /*
     Displacement priority :
     1. Secondary bots have priority of displacement over main bot
     2. Among secondary bots, SB1 has priority over SB2
     3. Bots avoid collisions by stopping before collision and waiting for the other bot to pass
     */

    public void myMove() {
        // Correction d'odométrie par les murs avant de bouger
        correctOdometryWithWalls();

        // Check for WAITING_FOR_ALLY_STATUS task
        for (int i = 0; i < taskQueue.size(); i++) {
            if (taskQueue.get(i).task == Task.WAITING_FOR_ALLY_STATUS) {
                if (taskQueue.get(i).attr.isTargetWaitingStepsReached()) {
                    // Remove the waiting task
                    taskQueue.remove(i);
                    break;
                } else {
                    taskQueue.get(i).attr.incrementWaitingStep();
                    sendLogMessage("Waiting for ally status... Step " + taskQueue.get(i).attr.getCurrentStep());
                    return; // Do not move while waiting
                }
            }
        }

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

    private static final double ALLY_TOO_CLOSE_DISTANCE = 100; // ajuste (15-30)

    private boolean isAllyTooClose() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot
                    || obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {

                if (obj.getObjectDistance() <= ALLY_TOO_CLOSE_DISTANCE) {
                    // Ally too close, turn away
                    // Check if bot should be moving a bit
                    for (QueuedTask task : taskQueue) {
                        if (task.task == Task.MOVE_A_BIT) {
                            taskQueue.addFirst(new QueuedTask(Task.MOVE_A_BIT, new TaskAttribute(5)));
                            sendLogMessage("trying to move abit to avoid ally");
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
        // Correction d'odométrie à chaque step
        correctOdometryWithWalls();
        clampPositionToArena();

        // Vérifier si le robot est perdu
        checkIfLost();

        // For each messages received, process it
        ArrayList<String> messages = this.fetchAllMessages();
        if (accpetingNewMessages) {
            for (String message : messages) {
                if (receiveMessage(message)) {
                    return;
                }
            }
        }

        // Check for enemies and respond if any
        if (!isOccupied)
            checkForEnemiesAndRespond();

        // Execute current task if any
        if (!taskQueue.isEmpty()) {
            callNextTask();
        }
    }

    // LONGEMENT DES MURS
    private boolean turnLeft() {
        // Check if the turn is complete
        if (isHeadingReached(taskQueue.getFirst().attr.targetHeading)) {
            taskQueue.removeFirst();
            callNextTask();
            return false;
        } else {
            stepTurn(Parameters.Direction.LEFT);
            return true;
        }
    }

    private boolean turnRight() {
        // Check if the turn is complete
        if (isHeadingReached(taskQueue.getFirst().attr.targetHeading)) {
            taskQueue.removeFirst();
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
        taskQueue.removeFirst();
        callNextTask();
        return false;
    }

    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(dir1-dir2)<ANGLEPRECISION;
    }

    // RENDEZ-VOUS logic
    // Checking for messages
    private boolean receiveMessage(String message) {
        String[] parts = message.split(";");

        // ===== Messages de localisation pour robots perdus =====

        // Réception d'une demande de position d'un robot perdu
        if (parts[0].equals(REQUEST_POSITION) && parts.length == 2) {
            String requesterId = parts[1];
            // Répondre avec notre position si on n'est pas perdu nous-même
            if (!isLost) {
                broadcast(POSITION_RESPONSE + ";" + requesterId + ";" + myX + ";" + myY + ";" + myID);
                System.out.println("[LOST " + myID + "] Received position request from " + requesterId + ". Sent my position.");
            }
            return false; // Ne pas interrompre les autres traitements
        }

        // Réception d'une réponse de position
        if (parts[0].equals(POSITION_RESPONSE) && parts.length == 5) {
            String targetId = parts[1];
            // Vérifier que le message nous est destiné
            if (!targetId.equals(myID)) {
                return false;
            }
            try {
                double allyX = Double.parseDouble(parts[2]);
                double allyY = Double.parseDouble(parts[3]);
                String allyId = parts[4];

                System.out.println("[LOST " + myID + "] Received position from " + allyId + ": (" + allyX + ", " + allyY + ")");

                // Trouver cet allié sur le radar pour calculer notre position
                ArrayList<IRadarResult> radarResults = detectRadar();
                for (IRadarResult result : radarResults) {
                    if (result.getObjectType() == IRadarResult.Types.TeamMainBot ||
                        result.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                        // Utiliser cet allié pour recalibrer (on suppose que c'est lui qui a répondu)
                        recalibrateFromAlly(allyX, allyY, result.getObjectDirection(), result.getObjectDistance());
                        return true;
                    }
                }
                // Allié plus sur le radar, ignorer
                System.out.println("[LOST " + myID + "] Ally no longer on radar, ignoring position response.");
            } catch (NumberFormatException e) {
                System.out.println("[LOST " + myID + "] Invalid position response format.");
            }
        }


        if (parts[0].equals(READY_TO_ATTACK_MESSAGE)) {
            alliedReadyToAttackCount += 1;
            sendLogMessage("Ally ready to attack count: " + alliedReadyToAttackCount);
            if (alliedReadyToAttackCount >= 3) {
                alliedReadyToAttackCount = 0;
                sendLogMessage("Ready to roam !");
                // Roam to detect enemies
                taskQueue.clear();
                taskQueue.add(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
                return false;
            }
        }

        // ===== Autres messages =====

        if (parts[0].equals(RDV_MESSAGE) && parts.length == 4) {
            try {
                rendezvousX = Double.parseDouble(parts[1]);
                rendezvousY = Double.parseDouble(parts[2]);
                rendezvousAngle = Double.parseDouble(parts[3]);
                isOccupied = true;
                taskQueue.clear();
                sendLogMessage("Received rendez-vous point at (" + rendezvousX + ", " + rendezvousY + ").");
                taskQueue.addFirst(new QueuedTask(Task.GET_INTO_FORMATION));
                return true;
            } catch (NumberFormatException e) {
                sendLogMessage("Invalid rendez-vous message format.");
            }
        }

        if (parts[0].equals(WHOAREYOU) && parts.length == 4) {
            String requesterId = parts[3];
            try {
                double queriedX = Double.parseDouble(parts[1]);
                double queriedY = Double.parseDouble(parts[2]);
                if (amI(queriedX, queriedY)) {
                    // Respond with my ID and movement status, targeted to requester
                    broadcast(IAM + ";" + myID + ";" + isMoving + ";" + requesterId);
                    sendLogMessage("Received WHOAREYOU message from " + requesterId + ". Responded with my ID.");
                    return true;
                }
            } catch (NumberFormatException e) {
                sendLogMessage("Malformed WHOAREYOU message.");
            }
        }

        if (parts[0].equals(IAM) && parts.length == 4) {
            // IAM format: IAM;senderId;senderIsMoving;recipientId
            String senderId = parts[1];
            boolean senderIsMoving = Boolean.parseBoolean(parts[2]);
            String recipientId = parts[3];
            // Vérifier que le message nous est bien destiné
            if (!recipientId.equals(myID)) {
                return false;
            }
            sendLogMessage("Received IAM message from " + senderId + ". Moving status: " + senderIsMoving);
            // Trouver la tâche WAITING_FOR_ALLY_STATUS et stocker la réponse
            for (int i = 0; i < taskQueue.size(); i++) {
                if (taskQueue.get(i).task == Task.WAITING_FOR_ALLY_STATUS) {
                    TaskAttribute attr = taskQueue.get(i).attr;
                    attr.setAllyResponse(senderIsMoving);
                    // marquer l'attente comme terminée pour déclencher suppression/prise de décision
                    attr.markWaitingComplete();
                    sendLogMessage("Stored ally status and marked waiting complete.");
                    break;
                }
            }
            return true;
        }
        return false;
    }

    public boolean amI(double posX, double posY) {
        // If position matches the given position (within a small margin),return true
        return Math.abs(myX - posX) < 5.0 && Math.abs(myY - posY) < 5.0;
    }

    private Task getNextTask() {
        if (taskQueue.size() < 2) {
            return null;
        }
        return taskQueue.get(1).task;
    }

    // Move forward until an obstacle is detected
    public boolean moveForward() {
        IFrontSensorResult frontSensorResult = this.detectFront();
        if (frontSensorResult.getObjectType().equals(IFrontSensorResult.Types.NOTHING)) {
            myMove();
            return true;
        } else {
            // Obstacle detected, stop moving
            taskQueue.removeFirst();
            sendLogMessage("Obstacle detected ahead. Stopping movement.");
            // Turn around
            double turnDirection = getHeading() + Math.PI + (Math.random() - 0.5) * Math.PI; // [-π/2, +π/2] autour de derrière
            taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(turnDirection)));
            return false;
        }
    }

    // Comportement bot secondaire :
    // 1.Scout for enemies in a predifined pattern :
    //    a. Move forward while scanning
    //    b. If wall detected, turn right/left (depending on SB1 or SB2)
    //    c. scan for enemies while turning
    //    d. Repeat
    //    e. Move forward while scanning
    // If at any point an enemy is detected : Broadcast its position and run away.
    // Wait for teammate to positions themselves at Rendez-vous point.
    // Get into formation with teammate and attack together.

    // SB1 detection have priority over SB2 detection.

    // If any coordinate is negative, then no rendez-vous point has been set yet.
    public void headTowardCoord() {
        // Déplacement en ligne droite vers (targetX, targetY)
        double dx = targetX - myX;
        double dy = targetY - myY;
        double distance = Math.hypot(dx, dy);

        // Seuil d'arrivée
        if (distance <= 5.0) {
            sendLogMessage("Reached target coordinates (" + targetX + ", " + targetY + ").");
            return;
        }

        // Angle vers la cible
        double directionToTarget = Math.atan2(dy, dx);
        Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(directionToTarget);
        if (turnDirection != null) {
            // Tourner vers la direction cible
            stepTurn(turnDirection);
        } else {
            // Face à la cible -> avancer en ligne droite
            myMove();
        }
    }

    private boolean isOccupied = false;

    private double rendezvousX;
    private double rendezvousY;
    private double rendezvousAngle;

    private double targetX;
    private double targetY;

    private double RDV_OFFSET_X = 60.0;
    private double RDV_OFFSET_Y = 100.0;

    private String RDV_MESSAGE = "RENDEZVOUS";

    // Formation :
    /*            M
                    \
                S====\====== (VISION)
             M ---|-->X      (X : detected using Radar) (| = rendez-vous point)
                S====/====== (VISION)
                    /
                  M
     */
    public boolean getIntoFormation() {
        // Définir la position cible selon le robot
        if (myID.equals(SB1)) {
            targetX = rendezvousX - RDV_OFFSET_X;
            targetY = rendezvousY - (RDV_OFFSET_Y + 30);
        } else {
            targetX = rendezvousX - RDV_OFFSET_X;
            targetY = rendezvousY + RDV_OFFSET_Y;
        }

        // Vérifier si la position est atteinte
        if (Math.abs(myX - targetX) < 5.0 && Math.abs(myY - targetY) < 5.0) {
            sendLogMessage("In formation position. Now waiting for others.");
            isOccupied = false;
            // Si la position est attinte, on attends les autres bots.
            // Faire face aux ennemis en attendant
            taskQueue.removeFirst();
            return false;
        }

        sendLogMessage("Heading to formation (" + targetX + ", " + targetY + ")");
        headTowardCoord();
        return true;
    }


    public boolean scoutBehavior() {
            if (myID.equals(SB1)) {
                // Make a rectangle on the top side
                taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
                taskQueue.addFirst(new QueuedTask(Task.TURN_LEFT, new TaskAttribute(getHeading() - 0.5 * Math.PI)));
                taskQueue.addFirst(new QueuedTask(Task.TURN_LEFT, new TaskAttribute(getHeading() - 0.5 * Math.PI)));
                taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
            } else {
                // Make a rectangle on the bottom side
                taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
                taskQueue.addFirst(new QueuedTask(Task.TURN_LEFT, new TaskAttribute(getHeading() + 0.5 * Math.PI)));
                taskQueue.addFirst(new QueuedTask(Task.TURN_LEFT, new TaskAttribute(getHeading() + 0.5 * Math.PI)));
                taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
            }
            // Démarrer l'exécution des tâches
        callNextTask();
        return false;
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

    private static int rendezVousCount = 0;
    private static final String FIGHTINHG_ENEMY_MESSAGE = "FIGHTING_ENEMY";

    public void checkForEnemiesAndRespond() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                // Enemy detected
                double enemyDirection = result.getObjectDirection();
                double enemyDistance = result.getObjectDistance();
                if (rendezVousCount < 1) {
                    rendezVousCount += 1;
                    // Decide a rendez-vous point 200 units away from enemy
                    double rendezvousDistance = enemyDistance + 200.0;
                    // Go in the opposite direction of the detected enemy (move away from it)
                    double oppositeDirection = enemyDirection + Math.PI;
                    double rendezvousX = myX + rendezvousDistance * Math.cos(oppositeDirection);
                    double rendezvousY = myY + rendezvousDistance * Math.sin(oppositeDirection);
                    double rendezVousAngle = enemyDirection;
                    broadcast(RDV_MESSAGE + ";" + rendezvousX + ";" + rendezvousY + ";" + rendezVousAngle);
                    taskQueue.clear();
                    taskQueue.addFirst(new QueuedTask(Task.GET_INTO_FORMATION));
                    sendLogMessage("Enemy detected during scouting.");
                }
                else {
                    // Sends location to ally bots
                    double enemyX = myX + enemyDistance * Math.cos(enemyDirection);
                    double enemyY = myY + enemyDistance * Math.sin(enemyDirection);
                    String enemyMessage = FIGHTINHG_ENEMY_MESSAGE + ";" + enemyX + ";" + enemyY + ";" + myID;
                    broadcast(enemyMessage);

                    // Turn around and run away
                    double awayDirection = enemyDirection + Math.PI;
                    Parameters.Direction turnDirection = getOptimalTurnDirectionWithHysteresis(awayDirection);
                    if (turnDirection != null) {
                        taskQueue.clear();
                        taskQueue.addFirst(new QueuedTask(Task.ROAM_AND_AVOID_ATTACKS));
                        taskQueue.addFirst(new QueuedTask(Task.MOVE_FORWARD));
                        taskQueue.addFirst(new QueuedTask(Task.TURN, new TaskAttribute(awayDirection)));
                    }
                }
            }
        }
    }

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
                    double awayDirection = result.getObjectDirection() + Math.PI / 2;
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
