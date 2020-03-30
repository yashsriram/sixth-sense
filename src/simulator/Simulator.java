package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;
import simulator.environment.Landmark;
import simulator.environment.LineSegment;
import simulator.robot.Robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;
import java.util.concurrent.ThreadLocalRandom;

public class Simulator {
    // Simulator settings
    public final static int CONTROL_FREQ = 1;
    public static final int LASER_SCAN_FREQUENCY = 10;

    // Graphics
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static int SCALE = 100;
    final PApplet applet;

    // Environment
    private List<Landmark> lines = new ArrayList<>();

    // Robot
    private Robot robot;

    // Multi-thread access
    final static Vec2 goalControl = Vec2.zero();
    final static Vec2 currentControl = Vec2.zero();
    private static final double LINEAR_VELOCITY_ERROR_LIMIT = 0.1;
    private static final double ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

    public Simulator(PApplet applet, String sceneFilepath) {
        this.applet = applet;
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
                applet,
                Double.parseDouble(poseTokens.get(3)),
                Vec3.of(
                        Double.parseDouble(poseTokens.get(0)),
                        Double.parseDouble(poseTokens.get(1)),
                        Double.parseDouble(poseTokens.get(2))
                ),
                true
        );

        // Every subsequent line in the file is a line segment
        for (int i = 1; i < fileContents.size(); ++i) {
            List<String> lineFeatureTokens = fileContents.get(i);
            // Make sure the line information is of size 4 (x1,y1,x2,y2)
            assert (lineFeatureTokens.size() == 4);
            Vec2 p1 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(0)), Double.parseDouble(lineFeatureTokens.get(1)));
            Vec2 p2 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(2)), Double.parseDouble(lineFeatureTokens.get(3)));
            lines.add(new LineSegment(applet, p1, p2));
        }

        // Fork off the main simulation loop
        Thread robotLoop = new Thread(this::robotLoop);
        robotLoop.start();
    }

    public Vec2 getCurrentControl() {
        Vec2 ret = Vec2.zero();
        synchronized (currentControl) {
            ret.set(currentControl);
        }
        return ret;
    }

    public void applyControl(Vec2 ctrl) {
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

        for (Landmark line : lines) {
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
                update(loopDt);
            }
            if (iteration % LASER_SCAN_FREQUENCY == 0) {
                robot.laser.updateLaserScan(robot, lines);
            }

            iteration++;
            try {
                Thread.sleep(loopDuration);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void draw() {
        // Draw landmarks
        for (Landmark l : lines) {
            l.draw();
        }
        // Draw robot
        robot.draw();
    }
}
