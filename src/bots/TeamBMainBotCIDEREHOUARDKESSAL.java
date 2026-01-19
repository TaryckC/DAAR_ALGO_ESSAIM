package bots;

import java.util.ArrayList;
import java.util.Random;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Bot;
import robotsimulator.Brain;

/**
 * Strategy (Sniper) : main bot qui privilégie la précision.
 * - Distance idéale de tir : 1000
 * - Distance de fuite : 600
 * - Fog-of-war : projection simple de la position ennemie
 * - Kiting : stop & turn pour viser, puis ajustement de distance
 * - Communication : envoi/écoute de TARGET:x:y:heading:speed
 */
public class TeamBMainBotCIDEREHOUARDKESSAL extends Brain {

    // Bot binding
    private Bot bot;

    private int myRank = 0;
    private boolean isLeader = false;
    private enum State {
        ATTACK,
        MOVE,
        AVOID_WRECK,
        WAIT,
        FOLLOW,
        MOVE_TO_TARGET
    }
    private State state = State.MOVE;
    private final static double HEADING_PRECISION = 0.05;

    private final static double DISPERSION_TIR = 0.05;
    private final static int CHANGE_DIR = 1000;
    private Random rand = new Random();
    private int aliveMainBot = 3;
    private int leaderMoveTimer = 0;

    private double targetHeading = 0;
    // Variables persistantes pour la cible
    private double targetX = -1;
    private double targetY = -1;
    private double targetDir = 0;
    private boolean hasTarget = false;

    private int targetLockTimer = 0; // Le compte à rebours
    private final static int LOCK_DURATION = 50; // On reste fixé 50 steps (environ 2-3 sec)
    private final static double SAME_TARGET_THRESHOLD = 200.0;

    @Override
    public void bind(Bot bot) {
        this.bot = bot;
        super.bind(bot);
    }

    @Override
    public void activate() {
        sendLogMessage("Strategy Sniper activée");
        int id = 0;
        for (IRadarResult o: detectRadar()) {
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) id += 1;
            if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH)) id += 2;

        }
        this.myRank = id;

        // DÉFINITION DU LEADER : On décide arbitrairement que le rank 3 est le chef
        // (A ajuster selon ton équipe : regarde les logs pour voir ton ID)
        if (this.myRank == 3) {
            isLeader = true;
        }

        sendLogMessage("Activé. Rank: " + myRank + " | Leader: " + isLeader);
    }

    @Override
    public void step() {

        if (getHealth() <= 0) return;

        // ---  DETECTION  ---
        ArrayList<IRadarResult> radar = detectRadar();

        ArrayList<String> messages = fetchAllMessages();

        hasTarget = false;
        if (checkWallTurnLeft()) {
            myMove(); // On applique la rotation immédiatement
            return;   // On s'arrête là pour ce tour, on ne réfléchit pas plus loin.
        }
        detectSecondaryBots(detectFront());

        IRadarResult localTarget = detectClosestTarget(radar);
        if (localTarget != null && localTarget.getObjectDistance() < 1000) {
            double absAngle = localTarget.getObjectDirection();
            this.targetX = bot.getX() + localTarget.getObjectDistance() * Math.cos(absAngle);
            this.targetY = bot.getY() + localTarget.getObjectDistance() * Math.sin(absAngle);
            this.targetDir = localTarget.getObjectDirection();
            this.hasTarget = true;

        }
        else{
            hasTarget = readRadioTargets(messages);
        }
        // 3. ACTION
        if (hasTarget) {
            performCombat(targetDir);
        } else {

            if(state ==State.MOVE_TO_TARGET){
                moveToTarget();
            }else {
                if (isLeader) {
                    // Le chef utilise SON radar pour compter les troupes
                    performLeaderLogic(radar);
                } else {
                    // Les suiveurs écoutent la radio pour trouver le chef
                    performFollowerLogic(messages, radar);
                }
            }
        }


    }


    // =======================
    // Helpers
    // =======================

    private void moveToTarget() {
        // 1. Calcul de la distance vers les coordonnées reçues (Scout)
        double dx = this.targetX - bot.getX();
        double dy = this.targetY - bot.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);

        // 2. Calcul de l'angle absolu vers la cible
        double angleVersCible = Math.atan2(dy, dx);


        if (dist > 1000) {
            System.out.println("Moving to target...");
            // ON FONCE !
            // On met à jour le cap pour myMove()
            this.targetHeading = angleVersCible;

            // On utilise ta méthode de déplacement fluide
            myMove();

        }
        // Si on est À PORTÉE (< 1000)
        else {
            // ON ATTAQUE !

            // Important : On force les variables pour que le step() prenne le relais au prochain tour
            this.targetDir = angleVersCible;

            // Optionnel : On peut tirer immédiatement pour ne pas perdre 1 tour
            performCombat(angleVersCible);
        }
    }
    /**
     * Evitement : murs (front sensor) et répulsion alliés (radar < 50px)
     * Retourne true si une action d'évitement a été faite (et le step() doit s'arrêter)
     */
    private boolean checkAndAvoidObstacles() {
        IFrontSensorResult front = detectFront();
        ArrayList<IRadarResult> radar = detectRadar();

        // Défensive : si mur devant -> demi-tour
        if (front != null && front.getObjectType() == IFrontSensorResult.Types.WALL) {
            // demi-tour
            stepTurn(Parameters.Direction.LEFT);
            stepTurn(Parameters.Direction.LEFT);
            moveBack();
            return true;
        }

        // Repulsion des alliés très proches (< 50)
        if (radar != null) {
            for (IRadarResult r : radar) {
                if (r.getObjectType() == IRadarResult.Types.TeamMainBot ||
                        r.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                    if (r.getObjectDistance() < 50) {
                        // s'éloigner dans la direction opposée
                        double absAngle = getHeading() + r.getObjectDirection();
                        double awayAngle = normalizeAngle(absAngle + Math.PI);
                        // tourner vers awayAngle puis avancer
                        turnTowards(awayAngle);
                        move();
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // ===================== MOUVEMENT / UTILITAIRES =====================

    private void turnTowards(double dir) {
        double delta = normalizeAngle(dir - getHeading());
        if (delta > 0) stepTurn(Parameters.Direction.RIGHT);
        else stepTurn(Parameters.Direction.LEFT);
    }


    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
    private boolean isFacing(double dir) {
        double delta = normalizeAngle(dir - getHeading());
        return Math.abs(delta) < 0.05;
    }

    private boolean isSameDirection(double dir1, double dir2){
        return Math.abs(normalizeAngle(dir1) - normalizeAngle(dir2)) < 0.1;
    }
    private IRadarResult detectClosestTarget(ArrayList<IRadarResult> radar) {
        double minDistance = 100000;
        IRadarResult bestTarget = null;

        // 1. On trouve d'abord LE MEILLEUR (le plus proche)
        for (IRadarResult r : radar) {
            if (r.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                System.out.println("detected");;
                if (r.getObjectDistance() < minDistance) {
                    minDistance = r.getObjectDistance();
                    bestTarget = r;
                }
            }
        }

        // 2. Si on a trouvé quelqu'un, ON CALCULE SES COORDONNÉES ET ON ENVOIE
        if (bestTarget != null) {
            double absoluteAngle =bestTarget.getObjectDirection();

            double enemyX = bot.getX() + bestTarget.getObjectDistance() * Math.cos(absoluteAngle);
            double enemyY = bot.getY() + bestTarget.getObjectDistance() * Math.sin(absoluteAngle);
            broadcast("TARGET:" + enemyX + ":" + enemyY);

            return bestTarget;
        }

        return null;
    }
    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }


    private boolean readRadioTargets(ArrayList<String> messages) {
        for (String msg : messages) {
            if (msg.startsWith("TARGET:")) {
                String[] parts = msg.split(":");
                this.targetX = Double.parseDouble(parts[1]);
                this.targetY = Double.parseDouble(parts[2]);
                this.targetDir = Math.atan2(targetY  - bot.getY(), targetX - bot.getX());
                return true; // On prend la première cible valide

            }
            if(msg.startsWith("SCOUT:")){
                String[] parts = msg.split(":");
                this.targetX = Double.parseDouble(parts[1]);
                this.targetY = Double.parseDouble(parts[2]);

                state = State.MOVE_TO_TARGET; // On s'arrête pour viser
                //this.hasTarget = true;
                return false ; // On a trouvé une cible via radio, on arrête le tour ici !
            }

        }
        return false;
    }



    private void performCombat(double dir) {

        double dist = Math.sqrt(Math.pow(targetX - bot.getX(), 2) + Math.pow(targetY - bot.getY(), 2));
        if(dist<1000){
            if (isFacing(dir)) {
                double angleAleatoire = (rand.nextDouble() - 0.5) * DISPERSION_TIR;
                fire(dir );
            } else {
                turnTowards(dir);
            }

        }
        else{
            myMove();
        }}


        /*private void performCombat(double dir) { // 'dir' est peut-être périmé ici
        // Recalcul frais de l'angle et de la distance
        double dx = targetX - bot.getX();
        double dy = targetY - bot.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);
        double freshDir = Math.atan2(dy, dx); // Angle tout neuf

        if (dist < 1000) {
            if (isFacing(freshDir)) { // Utilise freshDir
                double angleAleatoire = (rand.nextDouble() - 0.5) * DISPERSION_TIR;
                fire(freshDir + angleAleatoire); // Tire avec l'angle frais
            } else {
                turnTowards(freshDir);
            }
        } else {
            // Si on est trop loin, on avance vers la cible
            this.targetHeading = freshDir;
            myMove();
        }
    }*/

    public boolean avoidWreck(IFrontSensorResult frontSensorResult,ArrayList<IRadarResult> radar){
        if (radar == null|| frontSensorResult.getObjectType() != IFrontSensorResult.Types.Wreck) return false;
        for (IRadarResult r : radar) {
            if(r.getObjectType() == IRadarResult.Types.Wreck ){
                double relativeDir = r.getObjectDirection();

                if (relativeDir > 0) {
                    stepTurn(Parameters.Direction.RIGHT); // Obstacle à gauche -> Tourne droite
                } else {
                    stepTurn(Parameters.Direction.LEFT);  // Obstacle à droite -> Tourne gauche
                }
                // Et on avance (Esquive par le mouvement)
                return true;
            }
        }
        return false;
    }

    public void performLeaderLogic(ArrayList<IRadarResult> radar) {

        int countAlliesMainRobots = 0;
        for(IRadarResult r: radar){
            if(r.getObjectType() == IRadarResult.Types.TeamMainBot ){
                countAlliesMainRobots++;
            }
        }



        if(countAlliesMainRobots < aliveMainBot-1){
            broadcast("LEADER_INFO:" + bot.getX() + ":" + bot.getY() + ":" + getHeading()+":WAIT");
            return;
        }
        else {
            leaderMoveTimer++;
            if (leaderMoveTimer > CHANGE_DIR) {
                setNewRandomHeading();
                leaderMoveTimer = 0;
            }
            broadcast("LEADER_INFO:" + bot.getX() + ":" + bot.getY() + ":" + getHeading()+":MOVE");

            myMove();
        }
    }

    public void performFollowerLogic(ArrayList<String> messages,ArrayList<IRadarResult> radarResults) {

        for (String msg : messages) {
            if (msg.startsWith("LEADER_INFO:")) {
                String[] parts = msg.split(":");
                double leaderX = Double.parseDouble(parts[1]);
                double leaderY = Double.parseDouble(parts[2]);
                double leaderHeading = Double.parseDouble(parts[3]);
                String leaderState = parts[4];
                if(leaderState.equals("WAIT")){

                    // Calcul de l'angle vers le leader
                    double myX = bot.getX();
                    double myY = bot.getY();

                    double dist = Math.sqrt(Math.pow(leaderX - myX, 2) + Math.pow(leaderY - myY, 2));

                    if (dist < 200) {

                        return;
                    }

                    double angleToLeader = Math.atan2(leaderY - myY, leaderX - myX);

                    // Se diriger vers le leader
                    this.targetHeading = angleToLeader;
                    myMove();
                    return;
                }
                if(leaderState.equals("MOVE")){
                    // Suivre la direction du leader
                    this.targetHeading = leaderHeading;
                    myMove();
                }
                return; // On suit le premier leader entendu
            }
        }
    }
    public void setNewRandomHeading() {
        // 1. On définit de combien on veut tourner MINIMUM (ex: 90 degrés = PI/2)
        double minTurn = Math.PI / 2;

        // 2. On calcule un changement aléatoire supplémentaire (entre 0 et PI)
        // Cela donne un virage total compris entre 90° et 180° (demi-tour)
        double randomTurn = rand.nextDouble() * (Math.PI - minTurn);

        // 3. On calcule l'angle total du virage
        double totalTurn = minTurn + randomTurn;

        // 4. On applique ce virage soit à Gauche, soit à Droite (50/50)
        if (rand.nextBoolean()) {
            this.targetHeading = normalizeAngle(this.targetHeading + totalTurn);
        } else {
            this.targetHeading = normalizeAngle(this.targetHeading - totalTurn);
        }
    }

    public void myMove() {
        // 1. On s'oriente vers la direction cible
        if (!isHeadingReached(this.targetHeading)) {
            turnTowards(this.targetHeading);
        } else {
            // 2. On avance
            move();
        }
    }

    public void detectSecondaryBots(IFrontSensorResult sensor){
        if (sensor.getObjectType() == IFrontSensorResult.Types.TeamSecondaryBot) {

            broadcast("SECONDARY_BOT_DETECTED:" + targetHeading+":" + bot.getX() + ":" + bot.getY());
        }

    }

    private boolean checkWallTurnLeft() {
        IFrontSensorResult front = detectFront();

        // Si on détecte un MUR (et pas autre chose)
        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            // On tourne à GAUCHE
            this.targetHeading = normalizeAngle(getHeading() - (Math.PI / 2));
            //state = State.AVOID_WALL;
            // On retourne true pour dire "J'ai agi, arrête le tour"
            return true;
        }

        // Sinon, on n'a rien fait
        return false;
    }

    /*
    public void myFire(double direction, double distanceToEnemy) {
        // Calculate enemy coordinates (direction is RELATIVE from radar -> convert to absolute)
        if (distanceToEnemy != -1) {
            double absDir = normalizeAngle(getHeading() + direction);
            double ennemyX = myX + distanceToEnemy * Math.cos(absDir);
            double ennemyY = myY + distanceToEnemy * Math.sin(absDir);
            // Send broadcast message when firing
            broadcast(FIGHTINHG_ENEMY_MESSAGE + ";" + ennemyX + ";" + ennemyY + ";" + myID);
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
    }*/


}