package demos

import processing.core.PApplet
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator

fun main() {
    // Settings
    LaserSensor.DRAW_LASERS = false
    RANSACLeastSquares.DRAW_PARTITIONS = false
    Simulator.GHOST_MODE = true
    RANSACLeastSquares.DISCONTINUITY_THRESHOLD = 60.0
    Simulation.PERIODICAL_CLEAN_THRESHOLD = 5
    // Scene
    val appletArgs = arrayOf("demos.Simulation", "data/apartment.scn", "-700f", "500f")
    PApplet.main(appletArgs)
}
