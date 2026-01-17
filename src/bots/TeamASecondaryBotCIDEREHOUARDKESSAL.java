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


    private static final String SB1 = "Secondary-bot-1";
    private static final String SB2 = "Secondary-bot-2";

    private static final String MSG_ENEMY_FOUND = "ENEMY_MARKED";
    private static final String WHOAREYOU = "WHOAREYOU";
    private static final String IAM = "IAM";

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
    }

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

    private static final double TURN_HYSTERESIS = 0.05;
    private static final double SHOOTING_RANGE_OFFSET = 10.0;
    private static final double ANGLEPRECISION = 0.01;
    private final static double HEADING_PRECISION = 0.01;

    private LinkedList<Task> currentTasks;
    private LinkedList<TaskAttribute> currentTaskAttributes;

    private boolean accpetingNewMessages;
    private boolean isMoving;
    private boolean hasATarget; // Should be updated when a target dies

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

        currentTasks = new LinkedList<>();
        currentTaskAttributes = new LinkedList<>();
        accpetingNewMessages = true;
        isMoving = true;

    }

    // return true if there is a task being executed and executes it
    // Useful to chain tasks
    public boolean callNextTask() {
        if (!currentTasks.isEmpty()) {
            switch (currentTasks.getFirst()) {
                case MOVE_FORWARD:
                    //sendLogMessage("Moving forward.");
                    return moveForward();
                // TODO : Find a way to merge these similar tasks
                case SCOUTING_TASK:
                    // TODO : Implement scouting pattern
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
                case CLOSE_DISTANCE:
                    closeDistance();
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

    // Cherche l'intersection entre deux directions.
    // Si l'intersection est trop proche du robot, alors on considère que les directions se chevauchent.
    // Retourne true si collision probable, false si on peut passer
    // posX, posY : position de l'autre robot
    // otherDirection : direction de l'autre robot (angle en radians)
    public boolean doDirectionsOverlap(double otherX, double otherY, double otherDirection) {
        // Notre direction actuelle
        double myDirection = getHeading();

        // Vitesses des robots (supposées identiques pour les secondary bots)
        double mySpeed = Parameters.teamASecondaryBotSpeed;
        double otherSpeed = Parameters.teamASecondaryBotSpeed; // On suppose même vitesse

        // Vecteurs directionnels
        double myDirX = Math.cos(myDirection);
        double myDirY = Math.sin(myDirection);
        double otherDirX = Math.cos(otherDirection);
        double otherDirY = Math.sin(otherDirection);

        // Calcul du déterminant pour vérifier si les lignes sont parallèles
        double det = myDirX * (-otherDirY) - myDirY * (-otherDirX);

        // Si det ≈ 0, les trajectoires sont parallèles (pas d'intersection)
        if (Math.abs(det) < 0.001) {
            // Trajectoires parallèles - vérifier si elles sont proches
            double dx = otherX - myX;
            double dy = otherY - myY;
            double distanceToLine = Math.abs(dx * myDirY - dy * myDirX);
            // Si la distance perpendiculaire est petite, risque de collision latérale
            return distanceToLine < 50.0; // Rayon de sécurité
        }

        // Résoudre le système pour trouver le point d'intersection
        // myX + t1 * myDirX = otherX + t2 * otherDirX
        // myY + t1 * myDirY = otherY + t2 * otherDirY
        double dx = otherX - myX;
        double dy = otherY - myY;

        // Paramètre t1 : temps pour que NOTRE robot atteigne l'intersection
        double t1 = (dx * (-otherDirY) - dy * (-otherDirX)) / det;
        // Paramètre t2 : temps pour que l'AUTRE robot atteigne l'intersection
        double t2 = (dx * (-myDirY) - dy * (-myDirX)) / (-det);

        // Si t1 < 0, l'intersection est derrière nous (pas de collision)
        if (t1 < 0) {
            return false;
        }

        // Si t2 < 0, l'intersection est derrière l'autre robot (pas de collision)
        if (t2 < 0) {
            return false;
        }

        // Convertir les paramètres en "temps" réel (nombre de steps)
        double timeForUs = t1 / mySpeed;
        double timeForOther = t2 / otherSpeed;

        // Marge de sécurité en steps (temps pour traverser la zone de collision)
        double safetyMargin = 30.0; // ~30 steps de marge

        // Si on arrive bien AVANT l'autre robot (avec marge), on peut passer
        if (timeForUs + safetyMargin < timeForOther) {
            return false; // Pas de collision, on a le temps de passer
        }

        // Si l'autre arrive bien AVANT nous (avec marge), on doit attendre
        if (timeForOther + safetyMargin < timeForUs) {
            return true; // Collision probable si on avance, on doit céder le passage
        }

        // Sinon, les temps sont trop proches - risque de collision
        return true;
    }

    // Check if robot is heading toward an obsctacle, an allied bot or wall
    public boolean willCollideWithObstacle() {
        // Checks radar results to determine if elements nearby are moving toward our direction
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.TeamSecondaryBot || result.getObjectType() == IRadarResult.Types.TeamMainBot) {
                double distance = result.getObjectDistance();
                double direction = result.getObjectDirection();
                // Infer allie approximate position
                double allieX = myX + distance * Math.cos(direction);
                double allieY = myY + distance * Math.sin(direction);

                // Check if directions overlap
                if (doDirectionsOverlap(allieX, allieY, direction)) {
                    String message = WHOAREYOU + ";" + allieX + ";" + allieY + ";" + myID;
                    broadcast(message);
                    // Wait for answer
                    currentTasks.add(Task.WAITING_FOR_ALLY_STATUS);
                    currentTaskAttributes.add(new TaskAttribute(1));
                    return true;
                }
            }
        }
        return false;
    }

    public boolean shouldMove() {
        // Si aucune tâche d'attente pour statut allié, on peut bouger
        for (int i = 0; i < currentTasks.size(); i++) {
            if (currentTasks.get(i) == Task.WAITING_FOR_ALLY_STATUS) {
                TaskAttribute attr = currentTaskAttributes.get(i);
                // Si allié a répondu
                if (attr.getAllyResponse() != null) {
                    boolean allyMoving = attr.getAllyResponse();
                    // Si l'allié est en mouvement, on ne bouge pas (laisser passer)
                    if (allyMoving) {
                        sendLogMessage("Ally reports moving -> yield the way.");
                        return false;
                    } else {
                        // Alliée immobile -> on peut avancer; supprimer la tâche d'attente
                        sendLogMessage("Ally reports stopped -> proceed.");
                        currentTasks.remove(i);
                        currentTaskAttributes.remove(i);
                        return true;
                    }
                } else {
                    // pas de réponse encore : si timeout atteint, on supprime et on bouge
                    if (attr.isTargetWaitingStepsReached()) {
                        sendLogMessage("Ally did not respond in time -> proceed.");
                        currentTasks.remove(i);
                        currentTaskAttributes.remove(i);
                        return true;
                    } else {
                        // Toujours en attente
                        sendLogMessage("Still waiting for ally status... step " + attr.getCurrentStep());
                        return false;
                    }
                }
            }
        }
        return true;
    }

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

        if (isAllyTooClose()) {
            sendLogMessage("Ally too close! Stopping to avoid collision.");
            return;
        }

        myX+=Parameters.teamASecondaryBotSpeed*Math.cos(getHeading());
        myY+=Parameters.teamASecondaryBotSpeed*Math.sin(getHeading());
        move();
    }

    private static final double ALLY_TOO_CLOSE_DISTANCE = 100; // ajuste (15-30)

    private boolean isAllyTooClose() {
        for (IRadarResult obj : detectRadar()) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot
                    || obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {

                if (obj.getObjectDistance() <= ALLY_TOO_CLOSE_DISTANCE) {
                    return true;
                }
            }
        }
        return false;
    }

    public void step() {
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
        else {

        }

        // Execute current task if any
        if (!currentTasks.isEmpty()) {
            callNextTask();
            return;
        }
        sendLogMessage("Scouting...");
        scoutBehavior();
    }

    // LONGEMENT DES MURS
    private boolean turnLeft() {
        if (currentTaskAttributes.isEmpty()) {
            // targetHeading = getHeading() - Math.PI / 2;
            this.currentTaskAttributes.add(new TaskAttribute(getHeading() - 0.5 * Math.PI));
        }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttributes.getFirst().targetHeading)) {
            currentTasks.removeFirst();
            currentTaskAttributes.removeFirst();
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
            this.currentTaskAttributes.add(new TaskAttribute(getHeading() + 0.5 * Math.PI));
        }
        // Check if the turn is complete
        if (isHeadingReached(currentTaskAttributes.getFirst().targetHeading)) {
            currentTasks.removeFirst();
            currentTaskAttributes.removeFirst();
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
        currentTasks.removeFirst();
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
        if (parts[0].equals(RDV_MESSAGE) && parts.length == 4) {
            try {
                rendezvousX = Double.parseDouble(parts[1]);
                rendezvousY = Double.parseDouble(parts[2]);
                rendezvousAngle = Double.parseDouble(parts[3]);
                isOccupied = true;
                currentTasks.clear();
                currentTasks.addFirst(Task.GET_INTO_FORMATION);
                sendLogMessage("Received rendez-vous point at (" + rendezvousX + ", " + rendezvousY + ").");
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
            for (int i = 0; i < currentTasks.size(); i++) {
                if (currentTasks.get(i) == Task.WAITING_FOR_ALLY_STATUS) {
                    TaskAttribute attr = currentTaskAttributes.get(i);
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
        if (currentTasks.size() < 2) {
            return null;
        }
        return currentTasks.get(1);
    }

    // Move forward until an obstacle is detected
    public boolean moveForward() {
        IFrontSensorResult frontSensorResult = this.detectFront();
        if (frontSensorResult.getObjectType().equals(IFrontSensorResult.Types.NOTHING)) {
            myMove();
            return true;
        } else {
            // Obstacle detected, stop moving
            currentTasks.removeFirst();
            sendLogMessage("Obstacle detected ahead. Stopping movement.");
            callNextTask();
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
            sendLogMessage("In formation position.");
            isOccupied = false;
            // Si la position est attinte, on attends les autres bots.
            // Faire face aux ennemis en attendant
            currentTasks.removeFirst();
            currentTasks.add(Task.TURN_TOWARD_TARGET);
            return false;
        }

        sendLogMessage("Heading to formation (" + targetX + ", " + targetY + ")");
        headTowardCoord();
        return true;
    }


    public boolean scoutBehavior() {
            if (myID.equals(SB1)) {
                // Make a rectangle on the top side
                currentTasks.add(Task.MOVE_FORWARD);
                currentTasks.add(Task.TURN_LEFT);
                currentTasks.add(Task.TURN_LEFT);
                currentTasks.add(Task.MOVE_FORWARD);
            } else {
                // Make a rectangle on the bottom side
                currentTasks.add(Task.MOVE_FORWARD);
                currentTasks.add(Task.TURN_RIGHT);
                currentTasks.add(Task.TURN_RIGHT);
                currentTasks.add(Task.MOVE_FORWARD);
            }
            // Démarrer l'exécution des tâches
        callNextTask();
        return false;
    }

    public void checkForEnemiesAndRespond() {
        ArrayList<IRadarResult> radarResults = detectRadar();
        for (IRadarResult result : radarResults) {
            if (result.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    result.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                // Enemy detected
                double enemyDirection = result.getObjectDirection();
                double enemyDistance = result.getObjectDistance();
                // Decide a rendez-vous point 200 units away from enemy
                double rendezvousDistance = enemyDistance + 200.0;
                // Go in the opposite direction of the detected enemy (move away from it)
                double oppositeDirection = enemyDirection + Math.PI;
                double rendezvousX = myX + rendezvousDistance * Math.cos(oppositeDirection);
                double rendezvousY = myY + rendezvousDistance * Math.sin(oppositeDirection);
                double rendezVousAngle = enemyDirection;
                broadcast(RDV_MESSAGE + ";" + rendezvousX + ";" + rendezvousY + ";" + rendezVousAngle);
                currentTasks.clear();
                System.out.println("Current tasks : " + currentTasks);
                currentTasks.addFirst(Task.GET_INTO_FORMATION);
                sendLogMessage("Enemy detected during scouting.");
            }
        }
    }
}
