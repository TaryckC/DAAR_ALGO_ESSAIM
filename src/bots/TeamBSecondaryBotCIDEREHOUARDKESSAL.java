package bots;

import characteristics.IFrontSensorResult;
import characteristics.IRadarResult;
import characteristics.Parameters;
import robotsimulator.Bot;
import robotsimulator.Brain;

import java.util.ArrayList;
import java.util.Random;

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
    }
    public void moveAway(ArrayList<String> messages){
        for(String msg : messages) {
            if (msg.startsWith("MAIN_INFO:")) {
                String[] parts = msg.split(":");

                double dir = Double.parseDouble(parts[1]);
                double mainX = Double.parseDouble(parts[2]);
                double mainY = Double.parseDouble(parts[3]);

                double myX = bot.getX();
                double myY = bot.getY();

                double distance = Math.sqrt(Math.pow(mainX - myX, 2) + Math.pow(mainY - myY, 2));

                if (distance < 500) {
                    System.out.println("EVASION SCOUT ACTIVEE");
                    this.evasionHeading = normalizeAngle(dir + Math.PI / 2);
                    this.state = State.EVADING;
                    this.evasionTimer = EVASION_TIME;

                    return;
                }

            }
        }
        turnTowards(this.evasionHeading);
        move();
    }
    @Override
    public void step() {
        if (getHealth() <= 0) return;

        if(checkWallTurnLeft()) {
            //return;
        }

        ArrayList<IRadarResult> radar = detectRadar();
        ArrayList<String > messages = fetchAllMessages();
        moveAway(messages);
        scanEnvironment(radar);
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

    public void myMove() {
        if (!isHeadingReached(this.targetHeading)) {

            turnTowards(this.targetHeading);
        } else {
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

                double absoluteAngle = r.getObjectDirection();

                double enemyX = bot.getX() + r.getObjectDistance() * Math.cos(absoluteAngle);
                double enemyY = bot.getY() + r.getObjectDistance() * Math.sin(absoluteAngle);
                broadcast("SCOUT:" + enemyX + ":" + enemyY+":"+r.getObjectType());
            }
        }
    }

    private int moveAroundTarget(IRadarResult target) {
        if (target == null) return 1;
        double absAngle = target.getObjectDirection();
        double enemyX = bot.getX() + target.getObjectDistance() * Math.cos(absAngle);
        double enemyY = bot.getY() + target.getObjectDistance() * Math.sin(absAngle);

        broadcast("TARGET:" + enemyX + ":" + enemyY);
        double dist = target.getObjectDistance();

        double dirRelatif = target.getObjectDirection(); // Angle relatif

        if (dist < DIST_TOO_CLOSE) {
            return 2;
        }
        else if (dist > DIST_TOO_FAR) {
            this.targetHeading = normalizeAngle(getHeading() + dirRelatif);
            return 1;
        }
        else {
            return 0;
        }
    }

    private boolean checkWallTurnLeft() {
        IFrontSensorResult front = detectFront();

        if (front.getObjectType() != IFrontSensorResult.Types.NOTHING) {
            this.targetHeading = normalizeAngle(getHeading() - (Math.PI / 2));

            return true;
        }

        return false;
    }

}