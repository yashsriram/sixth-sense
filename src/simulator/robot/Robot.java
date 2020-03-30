package simulator.robot;

import math.Vec3;

public class Robot {
    public final static double MAX_LINEAR_ACCELERATION = 0.5;
    public final static double MAX_ANGULAR_ACCELERATION = 0.5;
    public final double robotLength;
    public final Vec3 truePose;

    public Robot(double robotLength, Vec3 truePose) {
        this.robotLength = robotLength;
        this.truePose = Vec3.of(truePose);
    }
}
