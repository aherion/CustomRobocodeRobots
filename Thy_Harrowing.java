package CustomRobots;

import robocode.*;

import java.awt.Color;
import java.awt.geom.*;
import java.util.*;


//Started with Alisdair Owen's anti-grav bot as template and customized to be much more effective
//My favorite addition to the template is that my bot can be equally as defensive but switch to being opportunistically aggressive

//Customizations include:

//Use of trig functions to calculate enemy and wall locations to avoid
//More trig to determine when my bot is in-between other bots and get out of the middle
//Made circular targeting system (uses enemy current location + movement speed to predict where to fire)
//Programmed bot to be more aggressive (can tell when enemy bot has below 30 energy and get very close with lvl 3 gun but not ram)

//Not yet implemented: trig to create line between two enemies and make my bot attracted to a point behind one of them to use them as a shield


public class Thy_Harrowing extends AdvancedRobot {


    //Start by initializing variables to be used: highlights include a hash table to store targets (key = name, value = enemy object),
    //as well as direction (1 = forward, -1 = backward), gun strength (1-3), strength of gravity point in middle of map,
    //robot, robot gun, and robot scanner all move independently of each other, and gun uses predictive fire
    Hashtable targets;
    Enemy target;
    final double PI = Math.PI;
    int direction = 1;
    double firePower;
    double midpointstrength = 0;
    int midpointcount = 0;

    public void run() {
        setAdjustRadarForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        setBodyColor(new Color(0, 0, 0));
        setGunColor(new Color(0, 0, 0));
        setRadarColor(new Color(175, 0, 0));
        setBulletColor(new Color(0, 175, 0));
        setScanColor(new Color(255, 0, 0));
        targets = new Hashtable();
        target = new Enemy();
        target.distance = 100000;
        setAdjustGunForRobotTurn(true);
        setAdjustRadarForGunTurn(true);
        turnRadarRightRadians(2 * PI);
        while (true) {
            antiGravMove();
            doFirePower();
            doScanner();
            doGun();
            out.println(target.distance);
            fire(firePower);
            execute();
        }
    }

    void antiGravMove() {

        //initialize force and force for x and y coordinates of later trig functions,
        //gravity point to be used for avoiding enemies/walls, enemy 1 and 2 variables, and enumerations for all enemies in above
        //hash table
        double force;
        double xforce = 0;
        double yforce = 0;
        double ang;
        GravPoint p;
        Enemy en;
        Enemy en2;
        Enumeration e = targets.elements();
        Enumeration e2;

        //while there are enemy elements stored the enemy to target is the next element and if that enemy is alive...
        while (e.hasMoreElements()) {
            en = (Enemy) e.nextElement();
            if (en.live) {
                //Swoop in for kill if my bots energy is more than enemy's energy + 20
                if (en.energy + 20 < getEnergy()) {
                    //new grav point on enemy to use as point of reference to head towards
                    p = new GravPoint(en.x, en.y, (getEnergy() - en.energy) * 1000);
                    force = p.power / Math.pow(getRange(getX(), getY(), p.x, p.y), 2);
                    //bearing from them to my bot
                    ang = normalizeBearing(Math.PI / 2 - Math.atan2(getY() - p.y, getX() - p.x));
                    //Add above force to total force based on bearing direction with trig
                    xforce += Math.sin(ang) * force;
                    yforce += Math.cos(ang) * force;

                    //avoid ramming by making new grav point that my bot is more repulsed by than the grav point initially used
                    //to send my bot towards the enemy so that my bot will never actually run into an enemy it is aggressing
                    p = new GravPoint(en.x, en.y, -(getEnergy() - en.energy) * 50000);
                    force = p.power / Math.pow(getRange(getX(), getY(), p.x, p.y), 3);
                    xforce += Math.sin(ang) * force;
                    yforce += Math.cos(ang) * force;
                } else {
                    //if there is no robot to bully (with the above if statement logic) then resume standard protocol of
                    //putting a grav point on enemy that will repulse my bot instead of attracting (aka stay away from strong bots)
                    p = new GravPoint(en.x, en.y, -1000);
                    force = p.power / Math.pow(getRange(getX(), getY(), p.x, p.y), 2);
                    ang = normalizeBearing(Math.PI / 2 - Math.atan2(getY() - p.y, getX() - p.x));
                    xforce += Math.sin(ang) * force;
                    yforce += Math.cos(ang) * force;
                }

                ang = normalizeBearing(Math.PI / 2 - Math.atan2(getY() - p.y, getX() - p.x));
                xforce += Math.sin(ang) * force;
                yforce += Math.cos(ang) * force;
                //triangulate enemy position and create vectors in front of an behind that position to avoid by splitting map into
                //4-quadrant grid, then using trig functions ascertaining enemy position in 1 of the 4 quadrants relative
                //to the 0-point of origin (note: this avoidance is subtle hence the initialization at 1000 but has high range due to
                //calculation at power of 4)
                xforce += 1000 / Math.pow(getRange(getX(), getY(), en.x, getY()), 4);
                xforce -= 1000 / Math.pow(getRange(getX(), getY(), 0, getY()), 4);
                xforce += 1000 / Math.pow(getRange(getX(), getY(), getBattleFieldWidth(), getY()), 4);
                xforce -= 1000 / Math.pow(getRange(getX(), getY(), en.x, getY()), 4);
                yforce += 1000 / Math.pow(getRange(getX(), getY(), getX(), en.y), 4);
                yforce -= 1000 / Math.pow(getRange(getX(), getY(), getX(), 0), 4);
                yforce += 1000 / Math.pow(getRange(getX(), getY(), getX(), getBattleFieldHeight()), 4);
                yforce -= 1000 / Math.pow(getRange(getX(), getY(), getX(), en.y), 4);
                e2 = targets.elements();
                while (e2.hasMoreElements()) {
                    en2 = (Enemy) e2.nextElement();
                    if (en2.live) {
                        //similar to above triangulation but to avoid vectors between two enemies (note: this avoidance is
                        //stronger than the above, hence initialization at 8000, but kicks in closer due to calculation at power of 3)
                        xforce += 8000 / Math.pow(getRange(getX(), getY(), en.x, getY()), 3);
                        xforce -= 8000 / Math.pow(getRange(getX(), getY(), en2.x, getY()), 3);
                        yforce += 8000 / Math.pow(getRange(getX(), getY(), getX(), en.y), 3);
                        yforce -= 8000 / Math.pow(getRange(getX(), getY(), getX(), en2.y), 3);
                    }
                }
            }
        }

        //create a grav point in center of map with random repulsion strength to establish base case for bot to move based on
        //so that bot is constantly on the move even when more specific movement methods are not in action
        midpointcount++;
        if (midpointcount > 5) {
            midpointcount = 0;
            midpointstrength = (Math.random() * 2000) - 1000;
        }
        p = new GravPoint(getBattleFieldWidth() / 2, getBattleFieldHeight() / 2, midpointstrength);
        force = p.power / Math.pow(getRange(getX(), getY(), p.x, p.y), 1.5);
        ang = normalizeBearing(Math.PI / 2 - Math.atan2(getY() - p.y, getX() - p.x));
        xforce += Math.sin(ang) * force;
        yforce += Math.cos(ang) * force;

        //wall avoidance behavior similar to enemy avoidance but with a small range (since walls don't move) and strong repulsion
        xforce += 10000 / Math.pow(getRange(getX(), getY(), getBattleFieldWidth(), getY()), 3);
        xforce -= 10000 / Math.pow(getRange(getX(), getY(), 0, getY()), 3);
        yforce += 10000 / Math.pow(getRange(getX(), getY(), getX(), getBattleFieldHeight()), 3);
        yforce -= 10000 / Math.pow(getRange(getX(), getY(), getX(), 0), 3);
        goTo(getX() - xforce, getY() - yforce);
    }

    //power of gun depends on distance from target (closer = more power)... added other conditions to use higher power in other methods
    private void doFirePower() {
        firePower = 400 / target.distance;
        if (firePower > 3) {
            firePower = 3;
        }
    }

    //simple move method to move towards an x any y coordinate
    private void goTo(double x, double y) {
        double dist = 20;
        double angle = Math.toDegrees(absBearing(getX(), getY(), x, y));
        double r = turnTo(angle);
        setAhead(dist * r);
    }

    //calculations to determine whether a left or right turn is would be quicker when going to turn before heading to a heading
    private int turnTo(double angle) {
        double ang;
        int dir;
        ang = normalizeBearing(getHeading() - angle);
        if (ang > 90) {
            ang -= 180;
            dir = -1;
        } else if (ang < -90) {
            ang += 180;
            dir = -1;
        } else {
            dir = 1;
        }
        setTurnLeft(ang);
        return dir;
    }

    //scanner always spins in circle (rapid scanning of entire map is absolutely crucial to this bots functionality)
    private void doScanner() {
        setTurnRadarLeftRadians(2 * PI);
    }

    //gun that retrieves enemy current position and current movement, estimates how long a shot will take to reach that distance,
    //and fires to a location ahead of enemy where they "should" be. Effective but has two major flaws: assumes the enemy will be moving
    //at a constant rate of speed and will maintain that heading
    private void doGun() {
        long time = getTime() + (int) Math.round((getRange(getX(), getY(), target.x, target.y) / (20 - (3 * firePower))));
        Point2D.Double p = target.guessPosition(time);

        //uses max gun on a target that is stationary (aka corner-hugger) based on trig calcs and bots that have 0 energy (aka disabled bots)
        if ((Math.round(p.y) == Math.round(target.y) || Math.round(p.x) == Math.round(target.x)) && target.energy > 0)
            firePower = 3;
        double gunOffset = getGunHeadingRadians() - (Math.PI / 2 - Math.atan2(p.y - getY(), p.x - getX()));
        setTurnGunLeftRadians(normalizeBearing(gunOffset));
    }

    //hard set the shortest angle to bearing if not between pi and -pi
    double normalizeBearing(double ang) {
        if (ang > PI)
            ang -= 2 * PI;
        if (ang < -PI)
            ang += 2 * PI;
        return ang;
    }

    //hard set the shortest angle to bearing if not between 0 and 2pi
    double normaliseHeading(double ang) {
        if (ang > 2 * PI)
            ang -= 2 * PI;
        if (ang < 0)
            ang += 2 * PI;
        return ang;
    }

    //calculate exact distance between two x's and y's using Pythagorean theorum
    public double getRange(double x1, double y1, double x2, double y2) {
        double xo = x2 - x1;
        double yo = y2 - y1;
        double h = Math.sqrt(xo * xo + yo * yo);
        return h;
    }

    //by treating map as a grid with 4 quadrants and using trigonometric functions we can establish the absolute bearing to enemy
    //instead of a relative one
    public double absBearing(double x1, double y1, double x2, double y2) {
        double xo = x2 - x1;
        double yo = y2 - y1;
        double h = getRange(x1, y1, x2, y2);
        if (xo > 0 && yo > 0) {
            return Math.asin(xo / h);
        }
        if (xo > 0 && yo < 0) {
            return Math.PI - Math.asin(xo / h);
        }
        if (xo < 0 && yo < 0) {
            return Math.PI + Math.asin(-xo / h);
        }
        if (xo < 0 && yo > 0) {
            return 2.0 * Math.PI - Math.asin(-xo / h);
        }
        return 0;
    }

    //when we scan another robot we either update their state in hash table they are stored in or add them if they aren't there already
    public void onScannedRobot(ScannedRobotEvent e) {
        Enemy en;
        if (targets.containsKey(e.getName())) {
            en = (Enemy) targets.get(e.getName());
        } else {
            en = new Enemy();
            targets.put(e.getName(), en);
        }
        //use absolute bearing method above to establish absolute bearing to each enemy scanned
        double absBearing_radian = (getHeadingRadians() + e.getBearingRadians()) % (2 * PI);
        //functionality to get name, heading, bearing, x and y coordinates, move speed, energy level, total distance from, and a time
        //of the scan so track what has happened between one scan and the next is possible
        en.name = e.getName();
        double h = normalizeBearing(e.getHeadingRadians() - en.heading);
        h = h / (getTime() - en.ctime);
        en.changehead = h;
        en.x = getX() + Math.sin(absBearing_radian) * e.getDistance();
        en.y = getY() + Math.cos(absBearing_radian) * e.getDistance();
        en.bearing = e.getBearingRadians();
        en.heading = e.getHeadingRadians();
        en.ctime = getTime();
        en.speed = e.getVelocity();
        en.distance = e.getDistance();
        en.energy = e.getEnergy();
        //if a newly scanned enemy is closer than the current targeted enemy target the closer enemy and if if the current
        //targeted enemy dies also switch to a new enemy
        en.live = true;
        if ((en.distance < target.distance) || (target.live == false)) {
            target = en;
        }
    }

    //allows hash table of stored enemies to be updated when one of them dies
    public void onRobotDeath(RobotDeathEvent e) {
        Enemy en = (Enemy) targets.get(e.getName());
        en.live = false;
    }
}

class Enemy {

    //uses of these are explained in comments above for onScannedRobot
    String name;
    double bearing, heading, speed, x, y, distance, changehead;
    long ctime;
    boolean live;
    double energy;

    Point2D.Double guessPosition(long when) {
        //time enemy was scanned, moment a shot is calculated to hit a target, and difference between these times
        double diff = when - this.ctime;
        double newX, newY;
        //try to predict when an enemy has changed heading to correct shot prediction (does not work very well)
        if (Math.abs(changehead) > 0.00001) {
            double radius = speed / changehead;
            double totalHead = diff * changehead;
            newY = y + (Math.sin(heading + totalHead) * radius) -
                    (Math.sin(heading) * radius);
            newX = x + (Math.cos(heading) * radius) -
                    (Math.cos(heading + totalHead) * radius);
        }
        //don't adjust shot prediction if the change in heading is minor (primarily based on the diff variable described above)
        else {
            newY = y + Math.cos(heading) * speed * diff;
            newX = x + Math.sin(heading) * speed * diff;
        }
        return new Point2D.Double(newX, newY);
    }
}

//setup x and y coordinates of a grav point and the strength of the repulsion of attraction
class GravPoint {
    public double x, y, power;

    public GravPoint(double pX, double pY, double pPower) {
        x = pX;
        y = pY;
        power = pPower;
    }
}
