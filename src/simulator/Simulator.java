package simulator;

import math.Vec2;
import math.Vec3;
import processing.core.PApplet;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Scanner;

public class Simulator {
    // Simulator settings
    public static int CONTROL_FREQ = 1;
    public static int LASER_SCAN_FREQUENCY = 10;

    // Graphics
    public static double SCALE = 100;
    private final PApplet applet;

    // Environment
    private final List<Landmark> lines = new ArrayList<>();

    // Robot
    private Robot robot;

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

        // Every line after 1st one in the file is a line segment
        for (int i = 1; i < fileContents.size(); ++i) {
            List<String> lineFeatureTokens = fileContents.get(i);
            // Make sure the line information is of size 4 (x1,y1,x2,y2)
            assert (lineFeatureTokens.size() == 4);
            Vec2 p1 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(0)), Double.parseDouble(lineFeatureTokens.get(1)));
            p1.scaleInPlace(SCALE);
            Vec2 p2 = Vec2.of(Double.parseDouble(lineFeatureTokens.get(2)), Double.parseDouble(lineFeatureTokens.get(3)));
            p2.scaleInPlace(SCALE);
            lines.add(new LineSegment(applet, p1, p2));
        }

        // Load robot pose
        List<String> poseTokens = fileContents.get(0);
        // Make sure the pose is of size 4 (x, y, th, radius)
        assert (poseTokens.size() == 4);
        // Fork off the main simulation loop
        robot = new Robot(
                applet,
                Double.parseDouble(poseTokens.get(3)) * SCALE,
                Vec3.of(
                        Double.parseDouble(poseTokens.get(0)) * SCALE,
                        Double.parseDouble(poseTokens.get(1)) * SCALE,
                        Double.parseDouble(poseTokens.get(2))
                ),
                true
        );
        Thread robotLoop = new Thread(this::robotLoop);
        robotLoop.start();
    }

    private void robotLoop() {
        final long loopDuration = 10;
        double loopDt = 1e-3 * loopDuration;
        int iteration = 0;

        while (robot.isRunning()) {
            if (iteration % CONTROL_FREQ == 0) {
                robot.updatePose(loopDt);
                for (Landmark line : lines) {
                    if (robot.isCrashing(line)) {
                        System.out.println("Robot: \"Oh No! I crashed!!!!\"");
                        robot.setRunning(false);
                    }
                }
            }
            if (iteration % LASER_SCAN_FREQUENCY == 0) {
                robot.updateSense(lines);
            }

            iteration++;
            try {
                Thread.sleep(loopDuration);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public List<Double> getLaserScan() {
        return robot.laser.getMeasurements();
    }

    public Vec2 getCurrentControl() {
        return robot.getCurrentControl();
    }

    public void applyControl(Vec2 control) {
        robot.applyControl(control);
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
