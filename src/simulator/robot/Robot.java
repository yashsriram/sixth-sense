package simulator.robot;

import math.Vec3;

public class Robot {
    public final static double MAX_LINEAR_ACCELERATION = 0.5;
    public final static double MAX_ANGULAR_ACCELERATION = 0.5;
    public final double robotLength;
    public final Vec3 truePose;
    private boolean isRunning;

    public Robot(double robotLength, Vec3 truePose, boolean isRunning) {
        this.robotLength = robotLength;
        this.truePose = Vec3.of(truePose);
        this.isRunning = isRunning;
    }

    public boolean isRunning() {
        return isRunning;
    }

    public void setRunning(boolean running) {
        isRunning = running;
    }
}
