package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Brain;
import robotsimulator.Bot;

import java.util.ArrayList;
import java.util.Random;

/**
 * Detect cible s'eloigne et envoi position aux MainBots
 */
public class TeamBSecondaryBotCIDEREHOUARDKESSAL extends Brain {
    private enum State {
        EVADING,
        MOVING_TO_TARGET,
        EXPLORING,
        AVOID_WALL
    }

    private State state = State.EXPLORING;

    private static int DIRECTION_TIMER = 300;
    private static int EVASION_TIME = 50;
    private int changeDirectionTimer = 0;
    // --- DISTANCES (Vision Radar ~550px) ---
    // On reste entre 350 et 530 pour voir sans se faire écraser.
    private final static double DIST_TOO_CLOSE = 350;
    private final static double DIST_TOO_FAR = 530;

    // --- VARIABLES GESTION ---
    private Random gen = new Random();
    private int evasiveManeuverTimer = 0;
    private boolean turningRight = true;
    private static final double DIST_FRIEND_TOO_CLOSE = 200;
    private final static double HEADING_PRECISION = 0.05;


    private Bot bot;

    private double evasionHeading = 0;
    private double targetHeading = 0;

    private int evasionTimer = 0;
    @Override
    public void bind(Bot bot) {
        this.bot = bot;
        super.bind(bot);
    }

    @Override
    public void activate() {
        sendLogMessage("Scout: Mode Spotter (Pas de tir).");
        turningRight = gen.nextBoolean();
    }

    @Override
    public void step() {
        if (getHealth() <= 0) return;
        if(checkWallTurnLeft()) {
            //return;
        }

        ArrayList<IRadarResult> radar = detectRadar();
        ArrayList<String > messages = fetchAllMessages();
        scanEnvironment(radar);
        fetchMainMessage(messages);
        if(state == State.EVADING) {
            if (evasionTimer > 0) {
                evasionTimer--;
                myMove();
                return;
            } else {
                state = State.EXPLORING;
            }
        }
        IRadarResult target = getBestTarget(radar);
        if (target != null) {
            state = State.MOVING_TO_TARGET;
        } else {
            state = State.EXPLORING;
        }

        if(state == State.EXPLORING){
            explore();
        }

        if(state == State.MOVING_TO_TARGET){
            int move = moveAroundTarget(target);
            if(move == 1){
                myMove();
            }else if(move == 2) {
                moveBack();
            }
            else {
                turnTowards(this.targetHeading);
            }

        }

        /*
        // 2. POLITESSE (Laisser passer les MainBots)
        if (avoidBlockingMainBots(radar)) {
            return;
        }

        // 3. OBSERVATION & BROADCAST
        IRadarResult target = getBestTarget(radar);

        if (target != null) {
            // A. BROADCAST (C'est notre seule arme !)
            double absAngle = getHeading() + target.getObjectDirection();
            double enemyX = bot.getX() + target.getObjectDistance() * Math.cos(absAngle);
            double enemyY = bot.getY() + target.getObjectDistance() * Math.sin(absAngle);
            broadcast("TARGET:" + enemyX + ":" + enemyY);

            // B. MOUVEMENT (Shadowing / Orbite)
            double dist = target.getObjectDistance();
            double dir = target.getObjectDirection(); // Angle relatif

            if (dist < DIST_TOO_CLOSE) {
                // Trop près ! DANGER -> On fuit à l'opposé
                turnTowards(getHeading() + dir + Math.PI);
                move();
            }
            else if (dist > DIST_TOO_FAR) {
                // Trop loin ! On risque de le perdre -> On se rapproche
                turnTowards(getHeading() + dir);
                move();
            }
            else {
                // Distance Parfaite (Zone Ninja) -> ON TOURNE AUTOUR
                // On se déplace à 90° de l'ennemi.
                // Ça permet de rester à distance tout en bougeant latéralement (dur à toucher).

                double strafeAngle = Math.PI / 2; // 90 degrés

                // Petit aléatoire pour ne pas être prévisible
                if (gen.nextBoolean()) strafeAngle += 0.2;
                else strafeAngle -= 0.2;

                turnTowards(getHeading() + dir + strafeAngle);
                move();
            }
            return;
        }

        // 4. EXPLORATION (Si personne en vue)
        if (gen.nextInt(20) == 0) {
            if (gen.nextBoolean()) stepTurn(Parameters.Direction.RIGHT);
            else stepTurn(Parameters.Direction.LEFT);
        }*/
    }

    // =========================================================
    // OUTILS (MURS, AMIS, CIBLAGE)
    // =========================================================

    private boolean checkAndAvoidWalls() {
        IFrontSensorResult front = detectFront();
        if (evasiveManeuverTimer > 0) {
            evasiveManeuverTimer--;
            if (turningRight) stepTurn(Parameters.Direction.RIGHT);
            else stepTurn(Parameters.Direction.LEFT);
            moveBack();
            return true;
        }
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            evasiveManeuverTimer = 8 + gen.nextInt(8);
            turningRight = gen.nextBoolean();
            return true;
        }
        return false;
    }

    private boolean avoidBlockingMainBots(ArrayList<IRadarResult> radar) {
        for (IRadarResult r : radar) {
            if (r.getObjectType() == IRadarResult.Types.TeamMainBot) {
                if (r.getObjectDistance() < DIST_FRIEND_TOO_CLOSE) {
                    double dirMain = r.getObjectDirection();
                    double fuiteRelatif = (normalizeAngle(dirMain) > 0) ?
                            dirMain - (Math.PI / 2) :
                            dirMain + (Math.PI / 2);

                    double headingDesire = normalizeAngle(getHeading() + fuiteRelatif);
                    double delta = normalizeAngle(headingDesire - getHeading());

                    // Stop & Turn (Pour bien dégager la voie)
                    if (Math.abs(delta) > 0.2) {
                        if (delta > 0) stepTurn(Parameters.Direction.RIGHT);
                        else stepTurn(Parameters.Direction.LEFT);
                    } else {
                        move();
                    }
                    return true;
                }
            }
        }
        return false;
    }

    private IRadarResult getBestTarget(ArrayList<IRadarResult> results) {
        IRadarResult best = null;
        double minDistance = Double.MAX_VALUE;

        for (IRadarResult r : results) {
            if (r.getObjectType() == IRadarResult.Types.OpponentMainBot ||
                    r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot) {
                if (r.getObjectDistance() < minDistance) {
                    minDistance = r.getObjectDistance();
                    best = r;
                }
            }
        }
        return best;
    }

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

    private void fetchMainMessage(ArrayList<String> messages) {
        for(String msg : messages) {
            if (msg.startsWith("SECONDARY_BOT_DETECTED:")) {
                String[] parts = msg.split(":");

                double dir = Double.parseDouble(parts[1]);
                double mainX = Double.parseDouble(parts[2]);
                double mainY = Double.parseDouble(parts[3]);
                // 2. Calcul de la distance (Théorème de Pythagore)
                double myX = bot.getX();
                double myY = bot.getY();

                // distance = RacineCarrée( (x2-x1)² + (y2-y1)² )
                double distance = Math.sqrt(Math.pow(mainX - myX, 2) + Math.pow(mainY - myY, 2));

                // 3. Condition de proximité
                if (distance < 100) {

                    // Ta logique d'évasion (déjà correcte)
                    this.targetHeading = normalizeAngle(dir + Math.PI / 2);
                    this.state = State.EVADING;
                    this.evasionTimer = EVASION_TIME;

                    return; // On a trouvé une urgence, on arrête de lire les autres messages
                } else {
                }

            }
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
    public void explore() {
        if (changeDirectionTimer > DIRECTION_TIMER) {
            targetHeading = gen.nextDouble() * 2 * Math.PI;
            changeDirectionTimer = 0;
        }
        changeDirectionTimer++;
        myMove();
    }

    private boolean isHeadingReached(double target) {
        return Math.abs(Math.sin(getHeading() - target)) < HEADING_PRECISION;
    }

    private void scanEnvironment(ArrayList<IRadarResult> radar) {
        for (IRadarResult r : radar) {
            if (r.getObjectType() == IRadarResult.Types.OpponentSecondaryBot || r.getObjectType() == IRadarResult.Types.OpponentMainBot) {

                // --- AJOUT : CALCUL ET ENVOI DE POSITION ---
                // 1. Calcul de l'angle absolu de l'ennemi (Mon Angle + Angle Relatif Ennemi)
                double absoluteAngle = r.getObjectDirection();

                // 2. Calcul des coordonnées (Trigonométrie)
                double enemyX = bot.getX() + r.getObjectDistance() * Math.cos(absoluteAngle);
                double enemyY = bot.getY() + r.getObjectDistance() * Math.sin(absoluteAngle);
                // 3. Envoi du message à l'équipe
                broadcast("SCOUT:" + enemyX + ":" + enemyY);
            }
        }
    }

    private int moveAroundTarget(IRadarResult target) {
        if (target == null) return 1;
        // 1. COMMUNICATION (Vital pour l'équipe)
        // On profite qu'on le voit pour crier sa position aux MainBots
        double absAngle = target.getObjectDirection();
        double enemyX = bot.getX() + target.getObjectDistance() * Math.cos(absAngle);
        double enemyY = bot.getY() + target.getObjectDistance() * Math.sin(absAngle);

        broadcast("TARGET:" + enemyX + ":" + enemyY);
        // 2. CALCUL DE MOUVEMENT (Shadowing)
        double dist = target.getObjectDistance();

        double dirRelatif = target.getObjectDirection(); // Angle relatif

        if (dist < DIST_TOO_CLOSE) {
            // CAS 1 : TROP PRÈS (< 350)
            // Danger ! On fuit à l'opposé (Demi-tour)
            // Angle actuel + Angle ennemi + PI (180°)
            return 2;
        }
        else if (dist > DIST_TOO_FAR) {
            // CAS 2 : TROP LOIN (> 530)
            // On risque de le perdre du radar -> On fonce sur lui
            this.targetHeading = normalizeAngle(getHeading() + dirRelatif);
            return 1;
        }
        else {
            return 0;
        }
    }

    private boolean checkWallTurnLeft() {
        IFrontSensorResult front = detectFront();

        // Si on détecte un MUR (et pas autre chose)
        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            // On tourne à GAUCHE
            this.targetHeading = normalizeAngle(getHeading() - (Math.PI / 2));
            //state = State.AVOID_WALL;
            // On retourne true pour dire "J'ai agi, arrête le tour"
            return true;
        }

        // Sinon, on n'a rien fait
        return false;
    }
}