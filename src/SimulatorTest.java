import math.Vec2;
import math.Vec3;
import processing.core.PApplet;

import java.util.Vector;

public class SimulatorTest extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    public static int SCALE = 5;

    Simulator sim;

    public void settings() {
        size(WIDTH, HEIGHT, P2D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        String scene_name = "/home/pandu/school/10/Sensing&Estimation/project/data/apartment.scn";

        sim = new Simulator(scene_name);
    }

    public void draw() {
        background(0);
        stroke(1);

        // Check the sensors to see if there's new information
        OdometryData odom = sim.getOdometry();
        LaserScanData scan = sim.getLaserScan();

        // For visualization purposes only, you shouldn't use this in your localization
        Vec3 pose = sim.getTruePose();

        // Draw the building
        for (LineSegmentFeature l : sim.getLineFeatures()) {
            line((float) l.p1.x * SCALE + WIDTH / 2f, (float) l.p1.y * SCALE + HEIGHT / 2f, (float) l.p2.x * SCALE + WIDTH / 2f, (float) l.p2.y * SCALE + HEIGHT / 2f);
        }

        // Draw the Laser scan
        Vector<Vec2> lines = new Vector<>();
        Vec2 position = Vec2.of(pose.x, pose.y).scaleInPlace(SCALE).plusInPlace(Vec2.of(WIDTH / 2.0, HEIGHT / 2.0));
        Vec2 laserEnd = position.minus(
                Vec2.of(Math.cos(pose.z), Math.sin(pose.z)).scaleInPlace(0.5 * sim.robotLength * SCALE)
        );
        Vec2 otherEnd = position.plus(
                Vec2.of(Math.cos(pose.z), Math.sin(pose.z)).scaleInPlace(0.5 * sim.robotLength * SCALE)
        );
        for (int i = 0; i < scan.distances.size(); ++i) {
            if (scan.distances.get(i) == Simulator.LASER_INVALID_MEASUREMENT) {
                continue;
            }
            double percentage = i / (Simulator.NUM_LASERS - 1.0);
            double theta = Simulator.MIN_THETA + (Simulator.MAX_THETA - Simulator.MIN_THETA) * percentage;

            Vec2 scan_pt_i = laserEnd.plus(Vec2.of(Math.cos(theta + pose.z), Math.sin(theta + pose.z)).scaleInPlace(scan.distances.get(i) * SCALE));
            lines.add(scan_pt_i);
        }
        stroke(1, 0, 0);
        for (Vec2 l : lines) {
            line((float) laserEnd.x, (float) laserEnd.y, (float) l.x, (float) l.y);
        }

        // Draw robot
        stroke(1);
        line((float) laserEnd.x, (float) laserEnd.y, (float) otherEnd.x, (float) otherEnd.y);

        surface.setTitle("Processing - FPS: " + Math.round(frameRate));
    }

    public void keyPressed() {
        if (key == 'p') {
            sim.sendControl(Vec2.zero());
        }
        if (keyCode == UP) {
            sim.sendControl(Vec2.of(10, 0));
        }
        if (keyCode == DOWN) {
            sim.sendControl(Vec2.of(-10, 0));
        }
        if (keyCode == LEFT) {
            sim.sendControl(Vec2.of(0, -0.5));
        }
        if (keyCode == RIGHT) {
            sim.sendControl(Vec2.of(0, 0.5));
        }
        if (key == '+') {
            SCALE++;
        }
        if (key == '-') {
            SCALE--;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"SimulatorTest"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
