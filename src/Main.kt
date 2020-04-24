import camera.QueasyCam
import extensions.plus
import extensions.timesAssign
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrixRMaj
import processing.core.PApplet
import processing.core.PConstants
import simulator.LaserSensor
import simulator.Robot
import simulator.Simulator
import java.awt.Point

data class TwoPoints(val point1: DMatrix2, val point2: DMatrix2)

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

    private fun getInferredLines() : MutableList<TwoPoints> {
        val inferredLines = mutableListOf<TwoPoints>()

        /* TODO fill the inferredLines with endpoints detected after RANSAC + Least Sqaures */
        val point1 = DMatrix2(1.0, 0.0)
        val point2 = DMatrix2(0.0, 1.0)
        inferredLines.add(TwoPoints(point1, point2))


        return inferredLines
    }

    override fun draw() {
        /* Update */
        val inferedLines = mutableListOf<TwoPoints>()

        /* Draw */
        background(0)
        stroke(1)
        sim!!.draw()
        surface.setTitle("Processing - FPS: " + Math.round(frameRate))
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
