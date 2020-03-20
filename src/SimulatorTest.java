import math.Vec2;
import math.Vec3;
import processing.core.PApplet;

import java.util.Vector;

public class SimulatorTest extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;

    Simulator sim;

    public void settings() {
        size(WIDTH, HEIGHT, P2D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();

        String scene_name = "/home/pandu/school/10/Sensing&Estimation/project/data/simple_rectangle_scaled.scn";

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
            line((float) l.p1.x, (float) l.p1.y, (float) l.p2.x, (float) l.p2.y);
        }

        // Draw the Laser scan:
        Vector<Vec2> lines = new Vector<>();
        Vec2 position = Vec2.of(pose.x, pose.y);
        Vec2 base = position.minus(
                Vec2.of(Math.cos(pose.z), Math.sin(pose.z)).scaleInPlace(0.5 * sim.robotLength)
        );
        lines.add(base);
        for (int i = 0; i < Simulator.NUM_LASERS; ++i) {
            if (scan.lengths.size() == 0 || scan.lengths.get(i) == Simulator.LASER_DIST_OVER_DIST_VAL) {
                continue;
            }
            double perc = i / (Simulator.NUM_LASERS - 1.0);
            double th = Simulator.MIN_THETA + (Simulator.MAX_THETA - Simulator.MIN_THETA) * perc;

            Vec2 scan_pt_i = base.plus(Vec2.of(Math.cos(th + pose.z), Math.sin(th + pose.z)).scaleInPlace(scan.lengths.get(i)));
            lines.add(scan_pt_i);
        }

        stroke(1, 0, 0);
        for (Vec2 l : lines) {
            line((float) l.x, (float) l.y, (float) position.x, (float) position.y);
        }

        surface.setTitle("Processing - FPS: " + Math.round(frameRate));
    }

    public void keyPressed() {
        if (key == 'p') {
            sim.sendControl(Vec2.zero());
        }
        if (keyCode == UP) {
            sim.sendControl(Vec2.of(10, 0));
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
