package simulator.robot.sensors;

import math.Vec2;
import simulator.environment.Landmark;
import simulator.robot.Robot;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ThreadLocalRandom;

public class Laser {
    public static final int COUNT = 181;

    // Angle
    public static final double MIN_THETA = -Math.PI / 2;
    public static final double MAX_THETA = Math.PI / 2;
    public static final double ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / COUNT;
    public static final double ANGLE_ERROR_LIMIT = 0.05;

    // Distance
    public static final double MAX_DISTANCE = 5.0;
    public static final double INVALID_MEASUREMENT_VALUE = MAX_DISTANCE + 1;
    public static final double DISTANCE_ERROR_LIMIT = 0.05;

    private final List<Double> measurements = new ArrayList<>(COUNT);

    public Laser() {
        for (int i = 0; i < COUNT; i++) {
            measurements.add(INVALID_MEASUREMENT_VALUE);
        }
    }

    public void updateLaserScan(Robot robot, List<Landmark> landmarks) {
        List<Double> newMeasurements = new ArrayList<>(Laser.COUNT);
        for (int i = 0; i < Laser.COUNT; i++) {
            newMeasurements.add(Laser.INVALID_MEASUREMENT_VALUE);
        }
        // Move the center of the scanner back from the center of the robot
        Vec2 truePosition = Vec2.of(robot.truePose.x, robot.truePose.y);
        Vec2 laserCenter = truePosition.minus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength));

        // For each laser beam
        for (int i = 0; i < Laser.COUNT; ++i) {
            double percentage = i / (Laser.COUNT - 1.0);
            double laserThErr = ThreadLocalRandom.current().nextDouble(-Laser.ANGLE_ERROR_LIMIT * Laser.ANGULAR_RESOLUTION, Laser.ANGLE_ERROR_LIMIT * Laser.ANGULAR_RESOLUTION);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage + robot.truePose.z + laserThErr;
            Vec2 v = Vec2.of(Math.cos(theta), Math.sin(theta));

            // Check intersection for each line feature
            for (Landmark landmark : landmarks) {
                double rayDistance = landmark.shortestRayDistance(laserCenter, v);
                if (rayDistance >= 0 && rayDistance < Laser.MAX_DISTANCE) {
                    newMeasurements.set(i, Math.min(rayDistance, newMeasurements.get(i)));
                }
            }

            // Add some noise to new measurements
            if (newMeasurements.get(i) < Laser.INVALID_MEASUREMENT_VALUE) {
                double laser_d_err = ThreadLocalRandom.current().nextDouble(-Laser.DISTANCE_ERROR_LIMIT, Laser.DISTANCE_ERROR_LIMIT);
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
