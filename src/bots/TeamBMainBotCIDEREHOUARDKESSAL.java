package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Bot;
import robotsimulator.Brain;

import java.util.ArrayList;
import java.util.Random;

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
    private double myX,myY;
    private final static double DISPERSION_TIR = 0.05;
    private final static int CHANGE_DIR = 4000;
    private Random rand = new Random();
    private int aliveMainBot = 3;
    private int leaderMoveTimer = 0;

    private double targetHeading = 0;

    // Variables persistantes pour la cible
    private double targetX = -1;
    private double targetY = -1;
    private double targetDir = 0;
    private boolean hasTarget = false;

    private int avoidTimer = 0;
    private boolean isAvoidingObstacles = false;// Le compte à rebours
    private final static int LOCK_DURATION_SEC = 200;
    private final static int LOCK_DURATION_MAIN = 500;

    private final static int AVOID_DURATION = 150;

    private int LOCK_DURATION = LOCK_DURATION_SEC;
    private int lockTimer = 0;
    private boolean lockCible = false;

    private int sendInfoTimer = 0;
    private static int INFO_BROADCAST_DELAY = 50;

    private double lastX = 0;
    private double lastY = 0;
    private int stuckCounter = 0;
    private boolean isUnblocking = false;
    private int unblockTimer = 0;
    private final static int STUCK_THRESHOLD = 800; // Si on ne bouge pas pendant 20 steps -> Bloqué
    private final static int UNBLOCK_DURATION = 200; // On fonce au hasard pendant 40 steps
    private double unblockHeading = 0;

    // Variable pour savoir si le leader est actuellement en train d'avancer ou d'attendre
    private boolean leaderIsMoving = false;

    // Tolérance : On ne s'arrête que si les suiveurs sont lâchés de plus de 450px
    private final static double DIST_TO_START = 300.0;
    private final static double DIST_TO_STOP = 450.0;

    @Override
    public void bind(Bot bot) {
        this.bot = bot;
        super.bind(bot);
    }

    @Override
    public void activate() {
        int id = 0;
        for (IRadarResult o: detectRadar()) {
            if (isSameDirection(o.getObjectDirection(), Parameters.NORTH)) id += 1;
            if (isSameDirection(o.getObjectDirection(), Parameters.SOUTH)) id += 2;

        }
        this.myRank = id;

        if (this.myRank == 3) {
            myX = Parameters.teamAMainBot2InitX;
            myY = Parameters.teamAMainBot2InitY;
            isLeader = true;
        }
        else if(this.myRank == 1){
            myX = Parameters.teamAMainBot3InitX;
            myY = Parameters.teamAMainBot3InitY;
        }
        else if(this.myRank == 2){
            myX = Parameters.teamAMainBot1InitX;
            myY = Parameters.teamAMainBot1InitY;
        }
        sendLogMessage("Activé. Rank: " + myRank + " | Leader: " + isLeader);
    }

    @Override
    public void step() {

        if (getHealth() <= 0) {
            broadcast("MAIN_BOT_DESTROYED:" + myRank+":"+isLeader);
            return;
        }

        if(sendInfoTimer > INFO_BROADCAST_DELAY){
            sendInfoTimer = 0;
            sendMainInfo();
        }
        sendMainInfo();

        ArrayList<IRadarResult> radar = detectRadar();

        ArrayList<String> messages = fetchAllMessages();
        fetchDeathMessages(messages);

        checkIfStuck();

        if (isUnblocking) {
            // Si on est en mode déblocage, on force le mouvement et on quitte le step
            this.targetHeading = unblockHeading;
            myMove();
            return;
        }

        hasTarget = false;
        if (checkWallTurnLeft()) {
            myMove(); // On applique la rotation immédiatement
            return;
        }
        if(processAvoidWallMessage(messages)){
            myMove();
            return;
        }

        if (!isAvoidingObstacles) {

            IRadarResult obstacle = getBlockingObjectMove(detectRadar());

            // Si on a trouvé un obstacle (robot ou mur détecté par radar)
            if (obstacle != null) {
                System.out.println("object devant");
                isAvoidingObstacles = true;
                avoidTimer = 0;

                // 2. LOGIQUE DE DECISION GAUCHE / DROITE
                // getObjectDirection() > 0 : L'obstacle est à ma GAUCHE -> Je tourne à DROITE (-PI/2)
                // getObjectDirection() < 0 : L'obstacle est à ma DROITE -> Je tourne à GAUCHE (+PI/2)

                if (obstacle.getObjectDirection() > 0) {
                    this.targetHeading = normalizeAngle(getHeading() - (Math.PI / 2));
                } else {
                    this.targetHeading = normalizeAngle(getHeading() + (Math.PI / 2));
                }
            }
        }
        if(isAvoidingObstacles){
            avoidTimer++;
            if(avoidTimer < AVOID_DURATION){
                myMove();
            }
            else{
                isAvoidingObstacles = false;
                avoidTimer = 0;
            }

            return;
        }

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

            if(state == State.MOVE_TO_TARGET){
                moveToTarget();
            }else {
                if (isLeader) {
                    // Le chef utilise SON radar pour compter les troupes
                    performLeaderLogic(radar,messages);
                } else {
                    // Les suiveurs écoutent la radio pour trouver le chef
                    performFollowerLogic(messages, radar);
                }
            }
        }


    }

    public void fetchDeathMessages(ArrayList<String> messages){
        int count = 0;
        for(String msg: messages){
            if(msg.startsWith("MAIN_BOT_DESTROYED:")){
                String[] parts = msg.split(":");
                int rankDestroyed = Integer.parseInt(parts[1]);
                boolean wasLeader = Boolean.parseBoolean(parts[2]);
                count++;
                if(wasLeader){
                    if(aliveMainBot == 2 && myRank == 1){
                        aliveMainBot--;
                        isLeader = true;
                    }
                    else{
                        aliveMainBot--;
                        isLeader = true;
                    }
                    sendLogMessage("Je suis le nouveau leader !");
                }
            }
        }
        aliveMainBot -= count;
    }


    private void moveToTarget() {
        double dx = this.targetX - bot.getX();
        double dy = this.targetY - bot.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);

        double angleVersCible = Math.atan2(dy, dx);

        double ofset = getRepulsionOffset(detectRadar());
        if (dist > 1000) {
            this.targetHeading = angleVersCible+ofset;
            myMove();
        }
        else {
            if(state == State.MOVE_TO_TARGET && !lockCible){
                state = State.ATTACK;
                lockTimer = 0;
                lockCible = true;
            }
            else{
                lockTimer++;
                if(lockTimer > LOCK_DURATION){
                    state = State.MOVE;
                    lockCible = false;
                }
            }

            performCombat(angleVersCible);
        }
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
            broadcast("TARGET:" + enemyX + ":" + enemyY+":"+bestTarget.getObjectType());

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
                String type = parts[3];
                LOCK_DURATION = type.equals("OpponentMainBot") ? LOCK_DURATION_MAIN : LOCK_DURATION_SEC;
                return true; // On prend la première cible valide

            }
            if(msg.startsWith("SCOUT:")){
                String[] parts = msg.split(":");
                this.targetX = Double.parseDouble(parts[1]);
                this.targetY = Double.parseDouble(parts[2]);
                String type = parts[3];
                LOCK_DURATION = type.equals("OpponentMainBot") ? LOCK_DURATION_MAIN : LOCK_DURATION_SEC;
                state = State.MOVE_TO_TARGET; // On s'arrête pour viser
                //this.hasTarget = true;
                return false ; // On a trouvé une cible via radio, on arrête le tour ici !
            }

        }
        return false;
    }
    private void performCombat(double dir) {
        // 1. Calculs de base
        double dx = targetX - bot.getX();
        double dy = targetY - bot.getY();
        double dist = Math.sqrt(dx * dx + dy * dy);
        double angleVersCible = Math.atan2(dy, dx); // Angle Absolu

        if (dist < 1000) {
            IRadarResult blocker = isLineOfFireBlocked(dist, angleVersCible, detectRadar());

            if (blocker != null) {

                // 2. CALCUL DU DÉCALAGE (STRAFE)
                // On recalcule la position relative de l'obstacle par rapport à la ligne de tir
                double angleToBlocker = blocker.getObjectDirection();
                double angleDiff = normalizeAngle(angleVersCible - angleToBlocker);

                // Si angleDiff > 0 : L'obstacle est à DROITE de la ligne de mire -> On va à GAUCHE
                // Si angleDiff < 0 : L'obstacle est à GAUCHE de la ligne de mire -> On va à DROITE

                if (angleDiff > 0) {
                    //  System.out.println("Obstacle à droite de la ligne de tir.");
                    // Obstacle à droite -> On se décale vers la GAUCHE de la cible (+90°)
                    this.targetHeading = normalizeAngle(angleVersCible + (Math.PI / 2));
                } else {
                    //System.out.println("Obstacle à droite de la ligne de tir.");

                    // Obstacle à gauche -> On se décale vers la DROITE de la cible (-90°)
                    this.targetHeading = normalizeAngle(angleVersCible - (Math.PI / 2));
                }

                // 3. ON BOUGE
                myMove();
                return;

            }
            if (isFacing(angleVersCible)) {
                double angleAleatoire = (rand.nextDouble() - 0.5) * DISPERSION_TIR;
                fire(angleVersCible);
            } else {
                turnTowards(angleVersCible);
            }
        } else {
            // Trop loin, on avance
            this.targetHeading = angleVersCible;

            myMove();
        }
    }


    public int countMainBotsInRange(double range ,ArrayList<String> messages){
        int count = 0;
        for(String msg: messages){
            if(msg.startsWith("MAIN_INFO:")){
                String[] parts = msg.split(":");
                double tX = Double.parseDouble(parts[1]);
                double  tY = Double.parseDouble(parts[2]);
                double dist = Math.sqrt(Math.pow(tX- bot.getX(), 2) + Math.pow(tY - bot.getY(), 2));
                int rank = Integer.parseInt(parts[4]);
                if(dist < range && rank != myRank){
                    count++;
                }
            }
        }
        return count;

    }

    public void performLeaderLogic(ArrayList<IRadarResult> radar, ArrayList<String> messages) {
        // 1. CHOIX DE LA DISTANCE (HYSTÉRÉSIS)
        // Si je suis déjà en mouvement, je suis tolérant (450).
        // Si je suis à l'arrêt, je suis strict (300).
        double checkRange = leaderIsMoving ? DIST_TO_STOP : DIST_TO_START;

        int countAlliesMainRobotsInRange = countMainBotsInRange(checkRange, messages);

        // Si pas assez d'alliés dans la zone définie
        if (countAlliesMainRobotsInRange < aliveMainBot - 1) {
            leaderIsMoving = false;

            broadcast("LEADER_INFO:" + bot.getX() + ":" + bot.getY() + ":" + getHeading() + ":WAIT");
            return;
        }
        else {
            leaderIsMoving = true;

            leaderMoveTimer++;
            if (leaderMoveTimer > CHANGE_DIR) {
                setNewRandomHeading();
                leaderMoveTimer = 0;
            }

            broadcast("LEADER_INFO:" + bot.getX() + ":" + bot.getY() + ":" + getHeading() + ":MOVE");
            myMove();
        }
    }

    public void sendMainInfo(){
        broadcast("MAIN_INFO:" + bot.getX() + ":" + bot.getY() + ":" + getHeading()+":"+myRank+":"+isLeader);
    }

    public void performFollowerLogic(ArrayList<String> messages,ArrayList<IRadarResult> radarResults) {

        for (String msg : messages) {
            if (msg.startsWith("LEADER_INFO:")) {
                String[] parts = msg.split(":");
                double leaderX = Double.parseDouble(parts[1]);
                double leaderY = Double.parseDouble(parts[2]);
                double leaderHeading = Double.parseDouble(parts[3]);
                String leaderState = parts[4];
                double repulsionOffset = getRepulsionOffset(radarResults);
                if(leaderState.equals("WAIT")){

                    // Calcul de l'angle vers le leader
                    double myX = bot.getX();
                    double myY = bot.getY();

                    double dist = Math.sqrt(Math.pow(leaderX - myX, 2) + Math.pow(leaderY - myY, 2));

                    if (dist < 300) {

                        return;
                    }

                    double angleToLeader = Math.atan2(leaderY - myY, leaderX - myX);

                    // Se diriger vers le leader
                    this.targetHeading = angleToLeader+repulsionOffset;
                    myMove();
                    return;
                }
                if(leaderState.equals("MOVE")){
                    // Suivre la direction du leader
                    this.targetHeading = leaderHeading+repulsionOffset;
                    myMove();
                }
                return;
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
        }

        else {
            myX += Parameters.teamAMainBotSpeed * Math.cos(getHeading());
            myY += Parameters.teamAMainBotSpeed * Math.sin(getHeading());
            move();
        }
    }

    private boolean checkWallTurnLeft() {
        IFrontSensorResult front = detectFront();

        if (front.getObjectType() == IFrontSensorResult.Types.WALL) {
            this.targetHeading = normalizeAngle(getHeading() - (Math.PI / 2));
            broadcast("AVOID_WALL:"+targetHeading+":" + bot.getX() + ":" + bot.getY());
            return true;
        }
        return false;
    }

    public boolean processAvoidWallMessage(ArrayList<String> messages) {
        for (String msg : messages) {
            if (msg.startsWith("AVOID_WALL:")) {
                String[] parts = msg.split(":");
                double avoidHeading = Double.parseDouble(parts[1]);
                double avoidX = Double.parseDouble(parts[2]);
                double avoidY = Double.parseDouble(parts[3]);

                // Calcul de la distance entre moi et le bot qui évite le mur
                double dx = avoidX - bot.getX();
                double dy = avoidY - bot.getY();
                double dist = Math.sqrt(dx * dx + dy * dy);

                // Si ce bot est proche de moi
                if (dist < 300) {
                    this.targetHeading = avoidHeading;
                    return true;
                }
            }
        }
        return false;
    }

    private IRadarResult getBlockingObjectMove(ArrayList<IRadarResult> radar) {

        double SAFETY_ANGLE = 0.40;

        double MAX_DIST = 100;

        IRadarResult closest = null;
        double minDist = Double.MAX_VALUE;

        for (IRadarResult r : radar) {

            if (r.getObjectType() == IRadarResult.Types.Wreck ||
                    r.getObjectType() == IRadarResult.Types.TeamSecondaryBot ) {
                if (Math.abs(r.getObjectDirection()) < SAFETY_ANGLE) {

                    if (r.getObjectDistance() < MAX_DIST) {

                        if (r.getObjectDistance() < minDist) {
                            minDist = r.getObjectDistance();
                            closest = r;
                        }
                    }
                }
            }
        }
        return closest;
    }
    private double getRepulsionOffset(ArrayList<IRadarResult> radar) {
        double deviation = 0;
        double MIN_SEPARATION = 150;
        double FORCE = 0.8;

        for (IRadarResult r : radar) {
            if (r.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    r.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {

                double dist = r.getObjectDistance();

                if (dist < MIN_SEPARATION) {
                    double urgency = (MIN_SEPARATION - dist) / MIN_SEPARATION;

                    // Si l'allié est à ma GAUCHE (angle > 0), je veux aller à DROITE (angle négatif)
                    if (r.getObjectDirection() > 0) {
                        deviation -= urgency * FORCE;
                    }
                    // Si l'allié est à ma DROITE (angle < 0), je veux aller à GAUCHE (angle positif)
                    else {
                        deviation += urgency * FORCE;
                    }
                }
            }
        }
        // On limite la déviation pour ne pas faire demi-tour complet (max PI/2)
        if (deviation > Math.PI/2) deviation = Math.PI/2;
        if (deviation < -Math.PI/2) deviation = -Math.PI/2;

        return deviation;
    }


    private IRadarResult isLineOfFireBlocked(double targetDistance, double fireAngle, ArrayList<IRadarResult> radar) {
        double BULLET_SIZE = 5.0;

        for (IRadarResult obj : radar) {
            if (obj.getObjectType() == IRadarResult.Types.TeamMainBot ||
                    obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot ||
                    obj.getObjectType() == IRadarResult.Types.Wreck) {

                double objRadius = 0;
                if (obj.getObjectType() == IRadarResult.Types.TeamSecondaryBot) {
                    objRadius = Parameters.teamASecondaryBotRadius; // ou environ 20
                } else {
                    objRadius = Parameters.teamAMainBotRadius; // ou environ 30
                }

                double distToObj = obj.getObjectDistance();

                // Angle Absolu de l'objet
                double angleToObj = obj.getObjectDirection();

                // Différence entre l'angle de tir et l'angle de l'objet
                double angleDiff = normalizeAngle(fireAngle - angleToObj);

                // Projection :
                // Lateral : distance perpendiculaire à la ligne de tir (sur le côté)
                double lateral = distToObj * Math.sin(angleDiff);
                // Forward : distance le long de la ligne de tir (devant)
                double forward = distToObj * Math.cos(angleDiff);

                // 3. La Condition de Blocage
                // - forward > 0 : L'objet est devant nous (pas derrière)
                // - forward < targetDistance : L'objet est AVANT l'ennemi (il cache la cible)
                // - Math.abs(lateral) < ... : L'objet est trop près de la ligne centrale du tir
                if (forward > 0 && forward < targetDistance && Math.abs(lateral) < (objRadius + BULLET_SIZE)) {

                    return obj;
                }
            }
        }
        return null;
    }

    private void checkIfStuck() {
        if (isUnblocking) {
            unblockTimer++;
            if (unblockTimer > UNBLOCK_DURATION) {
                isUnblocking = false; // Fini, on reprend la logique normale
                stuckCounter = 0;
            }
            return;
        }

        // Calcul de la distance parcourue depuis le dernier step
        double distMoved = Math.sqrt(Math.pow(bot.getX() - lastX, 2) + Math.pow(bot.getY() - lastY, 2));

        if (distMoved < 0.6) {
            stuckCounter++;
        } else {
            stuckCounter = 0;
        }

        // Si on est bloqué depuis trop longtemps
        if (stuckCounter > STUCK_THRESHOLD) {
            isUnblocking = true;
            unblockTimer = 0;

            unblockHeading = rand.nextDouble() * 2 * Math.PI;
        }

        // Mise à jour de la dernière position connue
        lastX = bot.getX();
        lastY = bot.getY();
    }
}