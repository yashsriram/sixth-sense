package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;
import simulator.environment.LineSegment;
import simulator.robot.Robot;
import simulator.robot.sensors.Laser;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.ThreadLocalRandom;

public class Simulator {
    // Graphics engine
    final PApplet parent;

    // Environment
    private List<LineSegment> lineFeatures = new ArrayList<>();

    // Laser scanner
    public final static Laser LASER_SENSOR = new Laser();

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
        List<List<String>> fileContents = new ArrayList<>();
        try {
            String delimiter = " ";
            File file = new File(sceneFilepath);
            Scanner scanner = new Scanner(file);
            while (scanner.hasNextLine()) {
                String line = scanner.nextLine();
                String[] tokens = line.split(delimiter);
                fileContents.add(new ArrayList<>(Arrays.asList(tokens)));
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
        List<String> poseTokens = fileContents.get(0);
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
            List<String> lineFeatureTokens = fileContents.get(i);
            // Make sure the line information is of size 4 (x1,y1,x2,y2)
            assert (lineFeatureTokens.size() == 4);
            Vec2 p1 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(0)), Double.parseDouble(lineFeatureTokens.get(1)));
            Vec2 p2 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(2)), Double.parseDouble(lineFeatureTokens.get(3)));
            lineFeatures.add(new LineSegment(p1, p2));
        }

        // Fork off the main simulation loop
        Thread robotLoop = new Thread(this::robotLoop);
        robotLoop.start();
    }

    public OdometryData getOdometryThreadSafe() {
        OdometryData ret;
        synchronized (CURRENT_ODOMETRY_DATA) {
            ret = CURRENT_ODOMETRY_DATA;
        }
        return ret;
    }

    public void applyControlThreadSafe(Vec2 ctrl) {
        synchronized (goalControl) {
            goalControl.set(ctrl);
        }
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

        for (LineSegment line : lineFeatures) {
            if (line.shortestDistance(Vec2.of(robot.truePose.x, robot.truePose.y)) < robot.robotLength) {
                System.out.println("Robot: \"Oh No! I crashed!!!!\"");
                robot.setRunning(false);
            }
        }
    }

    private void robotLoop() {
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
            if (iteration % Laser.LASER_SCAN_FREQUENCY == 0) {
                LASER_SENSOR.updateLaserScan(robot, lineFeatures);
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
        for (LineSegment l : lineFeatures) {
            parent.line((float) l.p1.x * scale + width / 2f, (float) l.p1.y * scale + height / 2f, (float) l.p2.x * scale + width / 2f, (float) l.p2.y * scale + height / 2f);
        }

        Vec2 position = Vec2.of(robot.truePose.x, robot.truePose.y).scaleInPlace(scale).plusInPlace(Vec2.of(width / 2.0, height / 2.0));
        Vec2 laserEnd = position.minus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength * scale));
        Vec2 otherEnd = position.plus(Vec2.of(Math.cos(robot.truePose.z), Math.sin(robot.truePose.z)).scaleInPlace(0.5 * robot.robotLength * scale));

        // Draw robot
        parent.stroke(1);
        parent.line((float) laserEnd.x, (float) laserEnd.y, (float) otherEnd.x, (float) otherEnd.y);

        // Draw lasers
        List<Double> distances = LASER_SENSOR.getLaserMeasurementsThreadSafe();
        List<Vec2> lines = new ArrayList<>(lineFeatures.size());
        for (int i = 0; i < distances.size(); ++i) {
            if (distances.get(i) == Laser.LASER_INVALID_MEASUREMENT) {
                continue;
            }
            double percentage = i / (Laser.LASER_COUNT - 1.0);
            double theta = Laser.MIN_THETA + (Laser.MAX_THETA - Laser.MIN_THETA) * percentage;

            Vec2 scan_pt_i = laserEnd.plus(Vec2.of(Math.cos(theta + robot.truePose.z), Math.sin(theta + robot.truePose.z)).scaleInPlace(distances.get(i) * scale));
            lines.add(scan_pt_i);
        }
        parent.stroke(1, 0, 0);
        for (Vec2 l : lines) {
            parent.line((float) laserEnd.x, (float) laserEnd.y, (float) l.x, (float) l.y);
        }
    }
}
