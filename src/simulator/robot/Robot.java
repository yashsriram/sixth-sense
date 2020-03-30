package simulator.robot;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;
import simulator.Simulator;
import simulator.robot.sensors.Laser;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public final static double MAX_LINEAR_ACCELERATION = 0.5;
    public final static double MAX_ANGULAR_ACCELERATION = 0.5;

    private final PApplet applet;

    public final double robotLength;
    public final Vec3 truePose;
    private boolean isRunning;

    public final Laser laser;

    public Robot(PApplet applet, double robotLength, Vec3 truePose, boolean isRunning) {
        this.applet = applet;
        this.robotLength = robotLength;
        this.truePose = Vec3.of(truePose);
        this.isRunning = isRunning;
        this.laser = new Laser();
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

        // Draw robot body
        applet.stroke(1);
        applet.line((float) laserEnd.x, (float) laserEnd.y, (float) otherEnd.x, (float) otherEnd.y);

        // Draw lasers
        List<Double> distances = laser.getLaserMeasurementsThreadSafe();
        List<Vec2> lines = new ArrayList<>(Laser.COUNT);
        for (int i = 0; i < distances.size(); ++i) {
            if (distances.get(i) == Laser.INVALID_MEASUREMENT_VALUE) {
                continue;
            }
            double percentage = i / (Laser.COUNT - 1.0);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage;

            Vec2 scan_pt_i = laserEnd.plus(Vec2.of(Math.cos(theta + truePose.z), Math.sin(theta + truePose.z)).scaleInPlace(distances.get(i) * Simulator.SCALE));
            lines.add(scan_pt_i);
        }
        applet.stroke(1, 0, 0);
        for (Vec2 l : lines) {
            applet.line((float) laserEnd.x, (float) laserEnd.y, (float) l.x, (float) l.y);
        }
    }
}
