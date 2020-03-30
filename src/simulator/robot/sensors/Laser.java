package simulator.robot.sensors;

import math.Vec2;
import processing.core.PApplet;
import simulator.Simulator;
import simulator.environment.Landmark;

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

    // Graphics
    private final PApplet applet;

    // Multi-thread access
    private final List<Double> measurements = new ArrayList<>(COUNT);

    public Laser(PApplet applet) {
        this.applet = applet;
        for (int i = 0; i < COUNT; i++) {
            measurements.add(INVALID_MEASUREMENT_VALUE);
        }
    }

    public void updateLaserScan(Vec2 position, double orientation, List<Landmark> landmarks) {
        List<Double> newMeasurements = new ArrayList<>(Laser.COUNT);
        for (int i = 0; i < Laser.COUNT; i++) {
            newMeasurements.add(Laser.INVALID_MEASUREMENT_VALUE);
        }

        // For each laser beam
        for (int i = 0; i < Laser.COUNT; ++i) {
            double percentage = i / (Laser.COUNT - 1.0);
            double laserThErr = ThreadLocalRandom.current().nextDouble(-Laser.ANGLE_ERROR_LIMIT * Laser.ANGULAR_RESOLUTION, Laser.ANGLE_ERROR_LIMIT * Laser.ANGULAR_RESOLUTION);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage + orientation + laserThErr;
            Vec2 v = Vec2.of(Math.cos(theta), Math.sin(theta));

            // Check intersection for each line feature
            for (Landmark landmark : landmarks) {
                double rayDistance = landmark.shortestRayDistanceFrom(position, v);
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

    public List<Double> getMeasurements() {
        List<Double> currentMeasurements;
        synchronized (measurements) {
            currentMeasurements = new ArrayList<>(measurements);
        }
        return currentMeasurements;
    }

    public void draw(Vec2 position, double orientation) {
        List<Double> distances = getMeasurements();
        List<Vec2> lasers = new ArrayList<>(Laser.COUNT);
        for (int i = 0; i < distances.size(); ++i) {
            if (distances.get(i) == Laser.INVALID_MEASUREMENT_VALUE) {
                continue;
            }
            double percentage = i / (Laser.COUNT - 1.0);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage;

            Vec2 laserSprite = position.plus(Vec2.of(Math.cos(orientation + theta), Math.sin(orientation + theta)).scaleInPlace(distances.get(i) * Simulator.SCALE));
            lasers.add(laserSprite);
        }
        applet.stroke(1, 0, 0);
        for (Vec2 l : lasers) {
            applet.line((float) position.x, (float) position.y, (float) l.x, (float) l.y);
        }
    }
}
