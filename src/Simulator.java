import math.Vec2;
import math.Vec3;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Arrays;
import java.util.Scanner;
import java.util.Vector;
import java.util.concurrent.ThreadLocalRandom;

class LaserScanData {
    Vector<Double> lengths = new Vector<>();
    long scanTime = 0;
}

class OdometryData {
    Vec2 odom = Vec2.zero();
    long odomTime = 0;
}

public class Simulator {
    // Laser Scanner Parameters:
    final static int NUM_LASERS = 181;
    final static double MIN_THETA = -Math.PI / 2;
    final static double MAX_THETA = Math.PI / 2;
    final static double MAX_LASER_DISTANCE = 150.0;
    final static double LASER_DIST_OVER_DIST_VAL = 170.0;
    final static double LASER_ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / NUM_LASERS;

    // Robot Parameters:
    double robotLength = 0;
    private Vec3 truePose = Vec3.zero();
    private Vector<LineSegmentFeature> lineFeatures = new Vector<>();

    // Random Distributions
    private static final double LASER_ANGLE_ERROR_LIMIT = 0.5;
    private static final double LASER_DISTANCE_ERROR_LIMIT = 0.05;
    private static final double LINEAR_VELOCITY_ERROR_LIMIT = 0.1;
    private static final double ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

    // Laser Scanner Parameters
    int laser_scan_freq = 10;  // How many iterations to wait between updates
    final static LaserScanData currentLaserScan = new LaserScanData();

    // Odometry Data Parameters
    int control_freq = 1; // How many iterations to wait between updates
    final static OdometryData currentOdometryData = new OdometryData();

    //    std:: thread main_loop;
    private boolean running = true;

    // Robot Parameters:
    final static double MAX_LINEAR_ACCELERATION = 3;
    final static double MAX_ANGULAR_ACCELERATION = 3;
    final static Vec2 goalControl = Vec2.zero();
    final static Vec2 currentControl = Vec2.zero();

    private static Vector<Vector<String>> loadFileByToken(final String filepath, final int nSkip, final String delimiter) throws FileNotFoundException {
        Vector<Vector<String>> fileContents = new Vector<>();
        File file = new File(filepath);
        Scanner scanner = new Scanner(file);
        int nLinesRead = 0;
        while (scanner.hasNextLine()) {
            String line = scanner.nextLine();
            nLinesRead++;
            if (nLinesRead <= nSkip) {
                continue;
            }
            String[] tokens = line.split(delimiter);
            fileContents.add(new Vector<>(Arrays.asList(tokens)));
        }
        scanner.close();
        return fileContents;
    }

    private void loadRobotPose(Vector<String> poseTokens) {
        // Make sure the pose is of size 4 (x, y, th, radius)
        assert (poseTokens.size() == 4);
        truePose.set(
                Double.parseDouble(poseTokens.get(0)),
                Double.parseDouble(poseTokens.get(1)),
                Double.parseDouble(poseTokens.get(2))
        );
        robotLength = Double.parseDouble(poseTokens.get(3));
    }

    private void addLineFeature(Vector<String> lineFeatureTokens) {
        // Make sure the line information is of size 4 (x1,y1,x2,y2)
        assert (lineFeatureTokens.size() == 4);
        Vec2 p1 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(0)), Double.parseDouble(lineFeatureTokens.get(1)));
        Vec2 p2 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(2)), Double.parseDouble(lineFeatureTokens.get(3)));
        lineFeatures.add(new LineSegmentFeature(p1, p2));
    }

    public Simulator(String map_file) {
        Vector<Vector<String>> fileContents = null;
        try {
            fileContents = loadFileByToken(map_file, 0, " ");
        } catch (FileNotFoundException e) {
            e.printStackTrace();
            System.exit(-1);
        }
        // Make sure we've loaded a non-empty map file.  Note that the first line in the file is the robot position
        assert (fileContents.size() > 0);

        loadRobotPose(fileContents.get(0));

        // Every Subsequent line in the file is a line segment:
        for (int i = 1; i < fileContents.size(); ++i) {
            addLineFeature(fileContents.get(i));
        }

        // Fork off the main simulation loop
        Thread mainLoop = new Thread(this::mainLoop);
        mainLoop.start();
    }

    public Vector<LineSegmentFeature> getLineFeatures() {
        return lineFeatures;
    }

    Vec3 getTruePose() {
        return truePose;
    }

    LaserScanData getLaserScan() {
        LaserScanData ret;
        synchronized (currentLaserScan) {
            ret = currentLaserScan;
        }
        return ret;
    }

    OdometryData getOdometry() {
        OdometryData ret;
        synchronized (currentOdometryData) {
            ret = currentOdometryData;
        }
        return ret;
    }

    void sendControl(Vec2 ctrl) {
        synchronized (goalControl) {
            goalControl.set(ctrl);
        }
    }

    private Vector<Double> computeLaserScan() {
        Vector<Double> rayDistances = new Vector<>(NUM_LASERS);
        for (int i = 0; i < NUM_LASERS; i++) {
            rayDistances.add(LASER_DIST_OVER_DIST_VAL);
        }
        // Move the center of the scanner back from the center of the robot
        Vec2 true_position = Vec2.of(truePose.x, truePose.y);
        Vec2 scanner_center = true_position.minus(
                Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength)
        );
        for (int i = 0; i < NUM_LASERS; ++i) {
            double perc = i / (NUM_LASERS - 1.0);
            double laser_th_err = ThreadLocalRandom.current().nextDouble(-LASER_ANGLE_ERROR_LIMIT * LASER_ANGULAR_RESOLUTION, LASER_ANGLE_ERROR_LIMIT * LASER_ANGULAR_RESOLUTION);
            double th = MIN_THETA + (MAX_THETA - MIN_THETA) * perc + truePose.z + laser_th_err;
            Vec2 v = Vec2.of(Math.cos(th), Math.sin(th));
//            System.out.println(th);
            for (LineSegmentFeature line : lineFeatures) {
                double rayDistance = line.checkIntersection(scanner_center, v);
                if (rayDistance >= 0 && rayDistance < MAX_LASER_DISTANCE) {
                    rayDistances.set(i, rayDistance);
                }
            }

            if (rayDistances.get(i) < LASER_DIST_OVER_DIST_VAL) {
                double laser_d_err = ThreadLocalRandom.current().nextDouble(-LASER_DISTANCE_ERROR_LIMIT, LASER_DISTANCE_ERROR_LIMIT);
                rayDistances.set(i, rayDistances.get(i) + laser_d_err);
            }
        }

        return rayDistances;
    }

    Vec3 contDynamics(Vec3 x_t, Vec2 u_t) {
        Vec3 x_dot_t = Vec3.zero();
        x_dot_t.x = u_t.x * Math.cos(x_t.z);
        x_dot_t.y = u_t.x * Math.sin(x_t.z);
        x_dot_t.z = u_t.y;
        return x_dot_t;
    }

    void updateCurrentControl(double dt) {
        Vec2 tmp_control = Vec2.zero();
        synchronized (currentControl) {
            // Only allow so much acceleration per timestep
            Vec2 control_diff = goalControl.minus(currentControl);
            if (Math.abs(control_diff.x) > MAX_LINEAR_ACCELERATION * dt) {
                control_diff.x = control_diff.x / Math.abs(control_diff.x) * MAX_LINEAR_ACCELERATION * dt;
            }
            if (Math.abs(control_diff.y) > MAX_ANGULAR_ACCELERATION * dt) {
                control_diff.y = control_diff.y / Math.abs(control_diff.y) * MAX_ANGULAR_ACCELERATION * dt;
            }
            currentControl.plusInPlace(control_diff);
            tmp_control = currentControl;
        }

        // Only apply noise if we're trying to move
        if (tmp_control.sqauredNorm() != 0.0) {
            tmp_control.x *= (1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT));
            tmp_control.y *= (1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT));
        }

        // Run the dynamics via RK4
        Vec3 k1, k2, k3, k4;
        Vec3 x2, x3, x4;
        k1 = contDynamics(truePose, tmp_control);
        x2 = truePose.plus(k1.scale(0.5f * dt));
        k2 = contDynamics(x2, tmp_control);
        x3 = truePose.plus(k2.scale(0.5f * dt));
        k3 = contDynamics(x3, tmp_control);
        x4 = truePose.plus(k3.scale(dt));
        k4 = contDynamics(x4, tmp_control);

        Vec3 dtrue_pose = Vec3.zero();
        dtrue_pose
                .plusInPlace(k1)
                .plusInPlace(k2.scale(2))
                .plusInPlace(k3.scale(2))
                .plusInPlace(k4)
                .scaleInPlace(dt / 6.0);
        truePose.plusInPlace(dtrue_pose);

//        System.out.println(true_pose);

        for (LineSegmentFeature line : lineFeatures) {
            if (line.shortestDistance(Vec2.of(truePose.x, truePose.y)) < robotLength) {
                System.out.println("Robot: \"Oh No! I crashed!!!!\"");
                running = false;
            }
        }
    }

    void mainLoop() {
        final long loop_duration = 10;
        double loop_dt = 1e-3 * loop_duration;

        int iter = 0;

        while (running) {
            if (iter % control_freq == 0) {
                // Do a control update
                updateCurrentControl(loop_dt);

                synchronized (currentOdometryData) {
                    synchronized (currentControl) {
                        currentOdometryData.odom = currentControl;
                    }
                    currentOdometryData.odomTime = System.currentTimeMillis();
                }
            }
            if (iter % laser_scan_freq == 0) {
                // Update the laser scan
                Vector<Double> tmp_scan = computeLaserScan();
                synchronized (currentLaserScan) {
                    currentLaserScan.lengths = tmp_scan;
                    currentLaserScan.scanTime = System.currentTimeMillis();
                }
            }

            iter++;
            try {
                Thread.sleep(loop_duration);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

}
