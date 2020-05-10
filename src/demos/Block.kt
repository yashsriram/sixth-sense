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
    RANSACLeastSquares.DISCONTINUITY_THRESHOLD = 200.0
    // Scene
    val appletArgs = arrayOf("demos.Simulation", "data/simple_block.scn", "900f", "500f")
    PApplet.main(appletArgs)
}
