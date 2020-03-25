package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;

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
    public final static int NUM_LASERS = 181;
    public final static double MIN_THETA = -Math.PI / 2;
    public final static double MAX_THETA = Math.PI / 2;
    public final static double LASER_MAX_DISTANCE = 5.0;
    public final static double LASER_INVALID_MEASUREMENT = LASER_MAX_DISTANCE + 1;
    public final static double LASER_ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / NUM_LASERS;
    public final static int LASER_SCAN_FREQ = 10;
    public final static LaserScanData CURRENT_LASER_SCAN = new LaserScanData();

    // Odometry data
    final static int CONTROL_FREQ = 1;
    final static OdometryData CURRENT_ODOMETRY_DATA = new OdometryData();

    // Robot parameters
    final static double MAX_LINEAR_ACCELERATION = 0.5;
    final static double MAX_ANGULAR_ACCELERATION = 0.5;
    public final double robotLength;
    private Vec3 truePose = Vec3.zero();
    final static Vec2 goalControl = Vec2.zero();
    final static Vec2 currentControl = Vec2.zero();
    private boolean running = true;

    // Randomness
    private static final double LASER_ANGLE_ERROR_LIMIT = 0.05;
    private static final double LASER_DISTANCE_ERROR_LIMIT = 0.05;
    private static final double LINEAR_VELOCITY_ERROR_LIMIT = 0.1;
    private static final double ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

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
        truePose.set(
                Double.parseDouble(poseTokens.get(0)),
                Double.parseDouble(poseTokens.get(1)),
                Double.parseDouble(poseTokens.get(2))
        );
        robotLength = Double.parseDouble(poseTokens.get(3));


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

    public Vec3 getTruePose() {
        return truePose;
    }

    public LaserScanData getLaserScan() {
        LaserScanData ret;
        synchronized (CURRENT_LASER_SCAN) {
            ret = CURRENT_LASER_SCAN;
        }
        return ret;
    }

    public OdometryData getOdometry() {
        OdometryData ret;
        synchronized (CURRENT_ODOMETRY_DATA) {
            ret = CURRENT_ODOMETRY_DATA;
        }
        return ret;
    }

    public void sendControl(Vec2 ctrl) {
        synchronized (goalControl) {
            goalControl.set(ctrl);
        }
    }

    private Vector<Double> computeLaserScan() {
        Vector<Double> measurements = new Vector<>(NUM_LASERS);
        for (int i = 0; i < NUM_LASERS; i++) {
            measurements.add(LASER_INVALID_MEASUREMENT);
        }
        // Move the center of the scanner back from the center of the robot
        Vec2 truePosition = Vec2.of(truePose.x, truePose.y);
        Vec2 laserCenter = truePosition.minus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength));

        // For each laser beam
        for (int i = 0; i < NUM_LASERS; ++i) {
            double percentage = i / (NUM_LASERS - 1.0);
            double laserThErr = ThreadLocalRandom.current().nextDouble(-LASER_ANGLE_ERROR_LIMIT * LASER_ANGULAR_RESOLUTION, LASER_ANGLE_ERROR_LIMIT * LASER_ANGULAR_RESOLUTION);
            double theta = MIN_THETA + (MAX_THETA - MIN_THETA) * percentage + truePose.z + laserThErr;
            Vec2 v = Vec2.of(Math.cos(theta), Math.sin(theta));

            // Check intersection for each line feature
            for (LineSegmentFeature line : lineFeatures) {
                double rayDistance = line.checkIntersection(laserCenter, v);
                if (rayDistance >= 0 && rayDistance < LASER_MAX_DISTANCE) {
                    measurements.set(i, Math.min(rayDistance, measurements.get(i)));
                }
            }

            // Add some noise to measurements
            if (measurements.get(i) < LASER_INVALID_MEASUREMENT) {
                double laser_d_err = ThreadLocalRandom.current().nextDouble(-LASER_DISTANCE_ERROR_LIMIT, LASER_DISTANCE_ERROR_LIMIT);
                measurements.set(i, measurements.get(i) + laser_d_err);
            }
        }

        return measurements;
    }

    Vec3 applyControl(Vec3 pose, Vec2 control) {
        Vec3 changeInPose = Vec3.zero();
        changeInPose.x = control.x * Math.cos(pose.z);
        changeInPose.y = control.x * Math.sin(pose.z);
        changeInPose.z = control.y;
        return changeInPose;
    }

    void updateCurrentControl(double dt) {
        Vec2 tmpControl = Vec2.zero();
        synchronized (currentControl) {
            // Only allow so much acceleration per timestep
            Vec2 control_diff = goalControl.minus(currentControl);
            if (Math.abs(control_diff.x) > MAX_LINEAR_ACCELERATION * dt) {
                control_diff.x = Math.signum(control_diff.x) * MAX_LINEAR_ACCELERATION * dt;
            }
            if (Math.abs(control_diff.y) > MAX_ANGULAR_ACCELERATION * dt) {
                control_diff.y = Math.signum(control_diff.y) * MAX_ANGULAR_ACCELERATION * dt;
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
        k1 = applyControl(truePose, tmpControl);
        x2 = truePose.plus(k1.scale(0.5f * dt));
        k2 = applyControl(x2, tmpControl);
        x3 = truePose.plus(k2.scale(0.5f * dt));
        k3 = applyControl(x3, tmpControl);
        x4 = truePose.plus(k3.scale(dt));
        k4 = applyControl(x4, tmpControl);

        Vec3 dtruePose = Vec3.zero();
        dtruePose
                .plusInPlace(k1)
                .plusInPlace(k2.scale(2))
                .plusInPlace(k3.scale(2))
                .plusInPlace(k4)
                .scaleInPlace(dt / 6.0);
        truePose.plusInPlace(dtruePose);

        for (LineSegmentFeature line : lineFeatures) {
            if (line.shortestDistance(Vec2.of(truePose.x, truePose.y)) < robotLength) {
                System.out.println("Robot: \"Oh No! I crashed!!!!\"");
                running = false;
            }
        }
    }

    void mainLoop() {
        final long loopDuration = 10;
        double loopDt = 1e-3 * loopDuration;
        int iteration = 0;

        while (running) {
            if (iteration % CONTROL_FREQ == 0) {
                // Do a control update
                updateCurrentControl(loopDt);
                synchronized (CURRENT_ODOMETRY_DATA) {
                    synchronized (currentControl) {
                        CURRENT_ODOMETRY_DATA.odom = currentControl;
                    }
                    CURRENT_ODOMETRY_DATA.odomTime = System.currentTimeMillis();
                }
            }
            if (iteration % LASER_SCAN_FREQ == 0) {
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
    }
}
