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
    RANSACLeastSquares.DISCONTINUITY_THRESHOLD = 120.0
    // Scene
    val appletArgs = arrayOf("demos.SLAM", "data/simple_block.scn")
    PApplet.main(appletArgs)
}
