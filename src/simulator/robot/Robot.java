package simulator.robot;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;
import simulator.Simulator;
import simulator.robot.sensors.Laser;

public class Robot {
    public final static double MAX_LINEAR_ACCELERATION = 0.5;
    public final static double MAX_ANGULAR_ACCELERATION = 0.5;

    // Graphics
    private final PApplet applet;

    // Main frame
    public final double robotLength;
    public Vec3 truePose;
    private boolean isRunning;

    // Sensors
    public final Laser laser;

    public Robot(PApplet applet, double robotLength, Vec3 truePose, boolean isRunning) {
        this.applet = applet;
        this.robotLength = robotLength;
        this.truePose = Vec3.of(truePose);
        this.isRunning = isRunning;
        this.laser = new Laser(applet);
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void setRunning(boolean running) {
        isRunning = running;
    }

    public void draw() {
        Vec2 position = Vec2.of(truePose.x, truePose.y).scaleInPlace(Simulator.SCALE).plusInPlace(Vec2.of(Simulator.WIDTH / 2.0, Simulator.HEIGHT / 2.0));
        Vec2 laserEnd = position.minus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength * Simulator.SCALE));
        Vec2 otherEnd = position.plus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength * Simulator.SCALE));

        // Draw lasers
        laser.draw(laserEnd, truePose.z);

        // Draw robot body
        applet.stroke(1);
        applet.line((float) laserEnd.x, (float) laserEnd.y, (float) otherEnd.x, (float) otherEnd.y);
    }
}
