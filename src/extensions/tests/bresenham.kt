package extensions.tests

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import processing.core.PApplet
import kotlin.math.sign
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator
import java.util.ArrayList
import demos.Simulation


fun main() {
    LaserSensor.DRAW_LASERS = false
    RANSACLeastSquares.DRAW_PARTITIONS = false
    Simulator.GHOST_MODE = true
    Simulation.DRAW_BRESENHAM_POINTS = true
//    val sensedEnds: MutableList<FMatrix2> = ArrayList(LaserSensor.COUNT)
    // Scene
//    bresenham2(FMatrix2(0.0F, 0.0F), FMatrix2(900F, 500F), sensedEnds)
    val appletArgs = arrayOf("demos.Simulation", "data/simple_block.scn", "900f", "500f")

    PApplet.main(appletArgs)
}