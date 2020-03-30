package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;

import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public class Robot {
    public static final double MAX_LINEAR_ACCELERATION = 0.5;
    public static final double MAX_ANGULAR_ACCELERATION = 0.5;
    public static final double LINEAR_VELOCITY_ERROR_LIMIT = 0.1;
    public static final double ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

    // Graphics
    private final PApplet applet;

    // Main frame
    private final double robotLength;
    private Vec3 truePose;
    private boolean isRunning;

    // Multi-thread access
    private final Vec2 goalControl = Vec2.zero();
    private final Vec2 currentControl = Vec2.zero();

    // Sensors
    final Laser laser;

    Robot(PApplet applet, double robotLength, Vec3 truePose, boolean isRunning) {
        this.applet = applet;
        this.robotLength = robotLength;
        this.truePose = Vec3.of(truePose);
        this.isRunning = isRunning;
        this.laser = new Laser(applet);
    }

    boolean isRunning() {
        return isRunning;
    }

    void setRunning(boolean running) {
        isRunning = running;
    }

    private static Vec3 getChangeInPose(Vec3 pose, Vec2 control) {
        Vec3 changeInPose = Vec3.zero();
        changeInPose.x = control.x * Math.cos(pose.z);
        changeInPose.y = control.x * Math.sin(pose.z);
        changeInPose.z = control.y;
        return changeInPose;
    }

    void updatePose(double dt) {
        Vec2 tmpControl = Vec2.zero();
        synchronized (currentControl) {
            synchronized (goalControl) {
                // Only allow so much acceleration per timestep
                Vec2 control_diff = goalControl.minus(currentControl);
                if (Math.abs(control_diff.x) > Robot.MAX_LINEAR_ACCELERATION * dt) {
                    control_diff.x = Math.signum(control_diff.x) * Robot.MAX_LINEAR_ACCELERATION * dt;
                }
                if (Math.abs(control_diff.y) > Robot.MAX_ANGULAR_ACCELERATION * dt) {
                    control_diff.y = Math.signum(control_diff.y) * Robot.MAX_ANGULAR_ACCELERATION * dt;
                }
                currentControl.plusInPlace(control_diff);
                tmpControl = currentControl;
            }
        }

        // Only apply noise if we're trying to move
        if (tmpControl.sqauredNorm() != 0.0) {
            tmpControl.x *= (1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT));
            tmpControl.y *= (1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT));
        }

        // Run the dynamics via RK4
        Vec3 k1, k2, k3, k4;
        Vec3 x2, x3, x4;
        k1 = getChangeInPose(truePose, tmpControl);
        x2 = truePose.plus(k1.scale(0.5f * dt));
        k2 = getChangeInPose(x2, tmpControl);
        x3 = truePose.plus(k2.scale(0.5f * dt));
        k3 = getChangeInPose(x3, tmpControl);
        x4 = truePose.plus(k3.scale(dt));
        k4 = getChangeInPose(x4, tmpControl);

        Vec3 dtruePose = Vec3.zero();
        dtruePose
                .plusInPlace(k1)
                .plusInPlace(k2.scale(2))
                .plusInPlace(k3.scale(2))
                .plusInPlace(k4)
                .scaleInPlace(dt / 6.0);
        truePose.plusInPlace(dtruePose);
    }

    void updateSense(List<Landmark> landmarks) {
        // Move the center of the scanner back from the center of the robot
        Vec2 truePosition = Vec2.of(truePose.x, truePose.y);
        Vec2 laserCenter = truePosition.minus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength));
        laser.updateLaserScan(laserCenter, truePose.z, landmarks);
    }

    boolean isCrashing(Landmark landmark) {
        return landmark.shortestDistanceFrom(Vec2.of(truePose.x, truePose.y)) < robotLength;
    }

    Vec2 getCurrentControl() {
        Vec2 answer = Vec2.zero();
        synchronized (currentControl) {
            answer.set(currentControl);
        }
        return answer;
    }

    void applyControl(Vec2 control) {
        synchronized (goalControl) {
            goalControl.set(control);
        }
    }

    void draw() {
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
