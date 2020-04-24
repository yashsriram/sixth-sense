import camera.QueasyCam
import extensions.minus
import extensions.plus
import extensions.timesAssign
import org.ejml.data.DMatrix2
import processing.core.PApplet
import processing.core.PConstants
import simulator.LaserSensor
import simulator.Robot
import simulator.Simulator
import java.util.ArrayList
import kotlin.math.roundToInt

class Main : PApplet() {
    companion object {
        const val WIDTH = 800
        const val HEIGHT = 800
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null

    override fun settings() {
        size(WIDTH, HEIGHT, PConstants.P3D)
    }

    override fun setup() {
        surface.setTitle("Processing")
        colorMode(PConstants.RGB, 1.0f)
        rectMode(PConstants.CENTER)
        noStroke()
        cam = QueasyCam(this)
        reset()
    }

    private fun reset() {
        val sceneName = "data/apartment.scn"
        // Play with these
        Simulator.SCALE = 100.0
        Simulator.CONTROL_FREQ = 1
        Simulator.LASER_SCAN_FREQUENCY = 10
        Robot.MAX_LINEAR_ACCELERATION = 20.0
        Robot.LINEAR_VELOCITY_ERROR_LIMIT = 2.0
        Robot.MAX_ANGULAR_ACCELERATION = 0.5
        Robot.ANGULAR_VELOCITY_ERROR_LIMIT = 0.1
        LaserSensor.ANGLE_ERROR_LIMIT = 0.05
        LaserSensor.DISTANCE_ERROR_LIMIT = 5.0
        sim = Simulator(this, sceneName)
    }

    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        val poseCopy = sim!!.truePose // FIXME: should use estimated pose here later
        val position = DMatrix2(poseCopy.a1, poseCopy.a2)
        val orientation = poseCopy.a3
        val centerToHead = DMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
        centerToHead *= 0.5 * sim!!.robotLength
        val tail = position - centerToHead

        val distances = sim!!.laserScan
        val laserEnds: MutableList<DMatrix2> = ArrayList(LaserSensor.COUNT)
        for (i in distances.indices) {
            if (distances[i] == LaserSensor.INVALID_MEASUREMENT) {
                continue
            }
            val percentage = i / (LaserSensor.COUNT - 1.0)
            val theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage
            val laserBeam = DMatrix2(kotlin.math.cos(orientation + theta),
                    kotlin.math.sin(orientation + theta))
            laserBeam *= distances[i]
            val laserEnd = tail + laserBeam
            laserEnds.add(laserEnd)
        }
        /* Visualization code - can be removed */
        stroke(1f, 1f, 0f)
        noFill()
        for (point in laserEnds) {
            circle3D(point.a1, point.a2, 1.0)
        }
        val inferedLines = getInferredLines(laserEnds)

        /* Draw */
        stroke(1)
        sim!!.draw()
        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}")
    }

    private fun circle3D(x: Double, z: Double, radius: Double) {
        beginShape()
        val resolution = 20
        for (i in 1..resolution) {
            val theta = 2 * PI / (resolution - 1) * i
            vertex((x + radius * cos(theta)).toFloat(), 0f, (z + radius * sin(theta)).toFloat())
        }
        endShape(CLOSE)
    }

    override fun keyPressed() {
        if (key == 'r') {
            reset()
        }
        if (key == 'p') {
            sim!!.applyControl(DMatrix2())
        }
        if (keyCode == PConstants.UP) {
            sim!!.applyControl(DMatrix2(100.0, 0.0))
        }
        if (keyCode == PConstants.DOWN) {
            sim!!.applyControl(DMatrix2(-100.0, 0.0))
        }
        if (keyCode == PConstants.LEFT) {
            sim!!.applyControl(DMatrix2(0.0, -0.5))
        }
        if (keyCode == PConstants.RIGHT) {
            sim!!.applyControl(DMatrix2(0.0, 0.5))
        }
        if (key == 'c') {
            cam!!.controllable = !cam!!.controllable
        }
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("Main")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
