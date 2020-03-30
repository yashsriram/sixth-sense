package simulator.robot.sensors;

import math.Vec2;
import simulator.environment.LineSegment;
import simulator.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public class Laser {
    public static final int LASER_COUNT = 181;
    public static final double MIN_THETA = -Math.PI / 2;
    public static final double MAX_THETA = Math.PI / 2;
    public static final double LASER_MAX_DISTANCE = 5.0;
    public static final double LASER_INVALID_MEASUREMENT = LASER_MAX_DISTANCE + 1;
    public static final double LASER_ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / LASER_COUNT;
    public static final double LASER_ANGLE_ERROR_LIMIT = 0.05;
    public static final double LASER_DISTANCE_ERROR_LIMIT = 0.05;
    public static final int LASER_SCAN_FREQUENCY = 10;

    private final List<Double> measurements = new ArrayList<>(LASER_COUNT);

    public Laser() {
        for (int i = 0; i < LASER_COUNT; i++) {
            measurements.add(LASER_INVALID_MEASUREMENT);
        }
    }

    public void updateLaserScan(Robot robot, List<LineSegment> lineFeatures) {
        List<Double> newMeasurements = new ArrayList<>(Laser.LASER_COUNT);
        for (int i = 0; i < Laser.LASER_COUNT; i++) {
            newMeasurements.add(Laser.LASER_INVALID_MEASUREMENT);
        }
        // Move the center of the scanner back from the center of the robot
        Vec2 truePosition = Vec2.of(robot.truePose.x, robot.truePose.y);
        Vec2 laserCenter = truePosition.minus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength));

        // For each laser beam
        for (int i = 0; i < Laser.LASER_COUNT; ++i) {
            double percentage = i / (Laser.LASER_COUNT - 1.0);
            double laserThErr = ThreadLocalRandom.current().nextDouble(-Laser.LASER_ANGLE_ERROR_LIMIT * Laser.LASER_ANGULAR_RESOLUTION, Laser.LASER_ANGLE_ERROR_LIMIT * Laser.LASER_ANGULAR_RESOLUTION);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage + robot.truePose.z + laserThErr;
            Vec2 v = Vec2.of(Math.cos(theta), Math.sin(theta));

            // Check intersection for each line feature
            for (LineSegment line : lineFeatures) {
                double rayDistance = line.rayDistance(laserCenter, v);
                if (rayDistance >= 0 && rayDistance < Laser.LASER_MAX_DISTANCE) {
                    newMeasurements.set(i, Math.min(rayDistance, newMeasurements.get(i)));
                }
            }

            // Add some noise to new measurements
            if (newMeasurements.get(i) < Laser.LASER_INVALID_MEASUREMENT) {
                double laser_d_err = ThreadLocalRandom.current().nextDouble(-Laser.LASER_DISTANCE_ERROR_LIMIT, Laser.LASER_DISTANCE_ERROR_LIMIT);
                newMeasurements.set(i, newMeasurements.get(i) + laser_d_err);
            }
        }

        // Update measurements
        synchronized (measurements) {
            for (int i = 0; i < newMeasurements.size(); i++) {
                measurements.set(i, newMeasurements.get(i));
            }
        }
    }


    public List<Double> getLaserMeasurementsThreadSafe() {
        List<Double> currentMeasurements;
        synchronized (measurements) {
            currentMeasurements = new ArrayList<>(measurements);
        }
        return currentMeasurements;
    }
}
