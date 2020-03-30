import camera.QueasyCam;
import math.Vec2;
import processing.core.PApplet;
import simulator.Laser;
import simulator.Robot;
import simulator.Simulator;

public class Main extends PApplet {
    public static final int WIDTH = 800;
    public static final int HEIGHT = 800;
    Simulator sim;
    QueasyCam cam;
    public void settings() {
        size(WIDTH, HEIGHT, P3D);
    }

    public void setup() {
        surface.setTitle("Processing");
        colorMode(RGB, 1.0f);
        rectMode(CENTER);
        noStroke();
        cam = new QueasyCam(this);
        reset();
    }

    private void reset() {
        String scene_name = "data/apartment.scn";
        // Play with these
        Simulator.SCALE = 100;
        Simulator.CONTROL_FREQ = 1;
        Simulator.LASER_SCAN_FREQUENCY = 10;

        Robot.MAX_LINEAR_ACCELERATION = 20;
        Robot.LINEAR_VELOCITY_ERROR_LIMIT = 2;

        Robot.MAX_ANGULAR_ACCELERATION = 0.5;
        Robot.ANGULAR_VELOCITY_ERROR_LIMIT = 0.1;

        Laser.ANGLE_ERROR_LIMIT = 0.05;
        Laser.MAX_DISTANCE = 500;
        Laser.DISTANCE_ERROR_LIMIT = 5;

        sim = new Simulator(this, scene_name);
    }

    public void draw() {
        background(0);
        stroke(1);
        sim.draw();
        surface.setTitle("Processing - FPS: " + Math.round(frameRate));
    }

    public void keyPressed() {
        if (key == 'r') {
            reset();
        }
        if (key == 'p') {
            sim.applyControl(Vec2.zero());
        }
        if (keyCode == UP) {
            sim.applyControl(Vec2.of(100, 0));
        }
        if (keyCode == DOWN) {
            sim.applyControl(Vec2.of(-100, 0));
        }
        if (keyCode == LEFT) {
            sim.applyControl(Vec2.of(0, -0.5));
        }
        if (keyCode == RIGHT) {
            sim.applyControl(Vec2.of(0, 0.5));
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[]{"Main"};
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
