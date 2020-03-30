package simulator.robot.sensors;

import java.util.ArrayList;
import java.util.List;

public class LaserSensor {
    public final static int NUM_LASERS = 181;
    public final static double MIN_THETA = -Math.PI / 2;
    public final static double MAX_THETA = Math.PI / 2;
    public final static double LASER_MAX_DISTANCE = 5.0;
    public final static double LASER_INVALID_MEASUREMENT = LASER_MAX_DISTANCE + 1;
    public final static double LASER_ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / NUM_LASERS;
    public final static int LASER_SCAN_FREQ = 10;
    public static final double LASER_ANGLE_ERROR_LIMIT = 0.05;
    public static final double LASER_DISTANCE_ERROR_LIMIT = 0.05;

    public List<Double> distances = new ArrayList<>();
    public long scanTime = 0;
}
