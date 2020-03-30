package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;
import simulator.environment.LineSegmentFeature;
import simulator.robot.Robot;
import simulator.robot.sensors.LaserSensor;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Scanner;
import java.util.Vector;
import java.util.concurrent.ThreadLocalRandom;

public class Simulator {
    // Graphics engine
    final PApplet parent;

    // Environment
    private Vector<LineSegmentFeature> lineFeatures = new Vector<>();

    // Laser scanner
    public final static LaserSensor CURRENT_LASER_SCAN = new LaserSensor();

    // Odometry data
    final static int CONTROL_FREQ = 1;
    final static OdometryData CURRENT_ODOMETRY_DATA = new OdometryData();
    final static Vec2 goalControl = Vec2.zero();
    final static Vec2 currentControl = Vec2.zero();
    private static final double LINEAR_VELOCITY_ERROR_LIMIT = 0.1;
    private static final double ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

    // Robot parameters
    private Robot robot;

    public Simulator(PApplet parent, String sceneFilepath) {
        this.parent = parent;
        // Read scene
        Vector<Vector<String>> fileContents = null;
        try {
            String delimiter = " ";
            fileContents = new Vector<>();
            File file = new File(sceneFilepath);
            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] tokens = line.split(delimiter);
                fileContents.add(new Vector<>(Arrays.asList(tokens)));
            }
            scanner.close();
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            System.exit(-1);
        }
        // Make sure we've loaded a non-empty map file.
        // Note that the first line in the file is the robot position
        assert (fileContents.size() > 0);

        // Load robot pose
        Vector<String> poseTokens = fileContents.get(0);
        // Make sure the pose is of size 4 (x, y, th, radius)
        assert (poseTokens.size() == 4);
        robot = new Robot(
                Double.parseDouble(poseTokens.get(3)),
                Vec3.of(
                        Double.parseDouble(poseTokens.get(0)),
                        Double.parseDouble(poseTokens.get(1)),
                        Double.parseDouble(poseTokens.get(2))
                ),
                true);

        // Every subsequent line in the file is a line segment
        for (int i = 1; i < fileContents.size(); ++i) {
            Vector<String> lineFeatureTokens = fileContents.get(i);
            // Make sure the line information is of size 4 (x1,y1,x2,y2)
            assert (lineFeatureTokens.size() == 4);
            Vec2 p1 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(0)), Double.parseDouble(lineFeatureTokens.get(1)));
            Vec2 p2 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(2)), Double.parseDouble(lineFeatureTokens.get(3)));
            lineFeatures.add(new LineSegmentFeature(p1, p2));
        }

        // Fork off the main simulation loop
        Thread mainLoop = new Thread(this::mainLoop);
        mainLoop.start();
    }

    public LaserSensor getLaserScanThreadSafe() {
        LaserSensor ret;
        synchronized (CURRENT_LASER_SCAN) {
            ret = CURRENT_LASER_SCAN;
        }
        return ret;
    }

    public OdometryData getOdometryThreadSafe() {
        OdometryData ret;
        synchronized (CURRENT_ODOMETRY_DATA) {
            ret = CURRENT_ODOMETRY_DATA;
        }
        return ret;
    }

    public void sendControlThreadSafe(Vec2 ctrl) {
        synchronized (goalControl) {
            goalControl.set(ctrl);
        }
    }

    private Vector<Double> computeLaserScan() {
        Vector<Double> measurements = new Vector<>(LaserSensor.NUM_LASERS);
        for (int i = 0; i < LaserSensor.NUM_LASERS; i++) {
            measurements.add(LaserSensor.LASER_INVALID_MEASUREMENT);
        }
        // Move the center of the scanner back from the center of the robot
        Vec2 truePosition = Vec2.of(robot.truePose.x, robot.truePose.y);
        Vec2 laserCenter = truePosition.minus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength));

        // For each laser beam
        for (int i = 0; i < LaserSensor.NUM_LASERS; ++i) {
            double percentage = i / (LaserSensor.NUM_LASERS - 1.0);
            double laserThErr = ThreadLocalRandom.current().nextDouble(-LaserSensor.LASER_ANGLE_ERROR_LIMIT * LaserSensor.LASER_ANGULAR_RESOLUTION, LaserSensor.LASER_ANGLE_ERROR_LIMIT * LaserSensor.LASER_ANGULAR_RESOLUTION);
            double theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage + robot.truePose.z + laserThErr;
            Vec2 v = Vec2.of(Math.cos(theta), Math.sin(theta));

            // Check intersection for each line feature
            for (LineSegmentFeature line : lineFeatures) {
                double rayDistance = line.checkIntersection(laserCenter, v);
                if (rayDistance >= 0 && rayDistance < LaserSensor.LASER_MAX_DISTANCE) {
                    measurements.set(i, Math.min(rayDistance, measurements.get(i)));
                }
            }

            // Add some noise to measurements
            if (measurements.get(i) < LaserSensor.LASER_INVALID_MEASUREMENT) {
                double laser_d_err = ThreadLocalRandom.current().nextDouble(-LaserSensor.LASER_DISTANCE_ERROR_LIMIT, LaserSensor.LASER_DISTANCE_ERROR_LIMIT);
                measurements.set(i, measurements.get(i) + laser_d_err);
            }
        }

        return measurements;
    }

    Vec3 getStateDerivative(Vec3 pose, Vec2 control) {
        Vec3 changeInPose = Vec3.zero();
        changeInPose.x = control.x * Math.cos(pose.z);
        changeInPose.y = control.x * Math.sin(pose.z);
        changeInPose.z = control.y;
        return changeInPose;
    }

    private void update(double dt) {
        Vec2 tmpControl = Vec2.zero();
        synchronized (currentControl) {
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

        // Only apply noise if we're trying to move
        if (tmpControl.sqauredNorm() != 0.0) {
            tmpControl.x *= (1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT));
            tmpControl.y *= (1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT));
        }

        // Run the dynamics via RK4
        Vec3 k1, k2, k3, k4;
        Vec3 x2, x3, x4;
        k1 = getStateDerivative(robot.truePose, tmpControl);
        x2 = robot.truePose.plus(k1.scale(0.5f * dt));
        k2 = getStateDerivative(x2, tmpControl);
        x3 = robot.truePose.plus(k2.scale(0.5f * dt));
        k3 = getStateDerivative(x3, tmpControl);
        x4 = robot.truePose.plus(k3.scale(dt));
        k4 = getStateDerivative(x4, tmpControl);

        Vec3 dtruePose = Vec3.zero();
        dtruePose
                .plusInPlace(k1)
                .plusInPlace(k2.scale(2))
                .plusInPlace(k3.scale(2))
                .plusInPlace(k4)
                .scaleInPlace(dt / 6.0);
        robot.truePose.plusInPlace(dtruePose);

        for (LineSegmentFeature line : lineFeatures) {
            if (line.shortestDistance(Vec2.of(robot.truePose.x, robot.truePose.y)) < robot.robotLength) {
                System.out.println("Robot: \"Oh No! I crashed!!!!\"");
                robot.setRunning(false);
            }
        }
    }

    private void mainLoop() {
        final long loopDuration = 10;
        double loopDt = 1e-3 * loopDuration;
        int iteration = 0;

        while (robot.isRunning()) {
            if (iteration % CONTROL_FREQ == 0) {
                // Do a control update
                update(loopDt);
                synchronized (CURRENT_ODOMETRY_DATA) {
                    synchronized (currentControl) {
                        CURRENT_ODOMETRY_DATA.odom = currentControl;
                    }
                    CURRENT_ODOMETRY_DATA.odomTime = System.currentTimeMillis();
                }
            }
            if (iteration % LaserSensor.LASER_SCAN_FREQ == 0) {
                // Update the laser scan
                Vector<Double> tmp_scan = computeLaserScan();
                synchronized (CURRENT_LASER_SCAN) {
                    CURRENT_LASER_SCAN.distances = tmp_scan;
                    CURRENT_LASER_SCAN.scanTime = System.currentTimeMillis();
                }
            }

            iteration++;
            try {
                Thread.sleep(loopDuration);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void draw(float scale, float width, float height) {
        // Draw the building
        for (LineSegmentFeature l : lineFeatures) {
            parent.line((float) l.p1.x * scale + width / 2f, (float) l.p1.y * scale + height / 2f, (float) l.p2.x * scale + width / 2f, (float) l.p2.y * scale + height / 2f);
        }

        Vec2 position = Vec2.of(robot.truePose.x, robot.truePose.y).scaleInPlace(scale).plusInPlace(Vec2.of(width / 2.0, height / 2.0));
        Vec2 laserEnd = position.minus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength * scale));
        Vec2 otherEnd = position.plus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength * scale));

        // Draw robot
        parent.stroke(1);
        parent.line((float) laserEnd.x, (float) laserEnd.y, (float) otherEnd.x, (float) otherEnd.y);

        // Draw lasers
        LaserSensor data = getLaserScanThreadSafe();
        Vector<Vec2> lines = new Vector<>();
        for (int i = 0; i < data.distances.size(); ++i) {
            if (data.distances.get(i) == LaserSensor.LASER_INVALID_MEASUREMENT) {
                continue;
            }
            double percentage = i / (LaserSensor.NUM_LASERS - 1.0);
            double theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage;

            Vec2 scan_pt_i = laserEnd.plus(Vec2.of(Math.cos(theta + robot.truePose.z), Math.sin(theta + robot.truePose.z)).scaleInPlace(data.distances.get(i) * scale));
            lines.add(scan_pt_i);
        }
        parent.stroke(1, 0, 0);
        for (Vec2 l : lines) {
            parent.line((float) laserEnd.x, (float) laserEnd.y, (float) l.x, (float) l.y);
        }
    }
}
