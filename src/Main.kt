import camera.QueasyCam
import extensions.circleXZ
import extensions.minus
import extensions.plus
import extensions.timesAssign
import org.ejml.data.DMatrix2
import processing.core.PApplet
import processing.core.PConstants
import simulator.LaserSensor
import simulator.Simulator
import java.util.*
import kotlin.math.roundToInt

class Main : PApplet() {
    companion object {
        const val WIDTH = 1000
        const val HEIGHT = 1000
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null
    private var hitGrid: HitGrid? = null

    override fun settings() {
        size(WIDTH, HEIGHT, PConstants.P3D)
    }

    override fun setup() {
        surface.setTitle("Processing")
        colorMode(PConstants.RGB, 1.0f)
        rectMode(PConstants.CENTER)
        cam = QueasyCam(this)
        reset()
    }

    private fun reset() {
        val sceneName = "data/apartment.scn"
        sim = Simulator(this, sceneName)
        hitGrid = HitGrid(this, DMatrix2(-1000.0, -1000.0), DMatrix2(1000.0, 1000.0), 1000, 1000)
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

        val distances = sim!!.laserDistances
        val laserEnds: MutableList<DMatrix2> = ArrayList(LaserSensor.COUNT)
        for (i in distances.indices) {
            if (distances[i] == LaserSensor.INVALID_DISTANCE) {
                continue
            }
            val percentage = i / (LaserSensor.COUNT - 1.0)
            val theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage
            val laserBeam = DMatrix2(kotlin.math.cos(orientation + theta),
                    kotlin.math.sin(orientation + theta))
            laserBeam *= distances[i]
            val laserEnd = tail + laserBeam
            laserEnds.add(laserEnd)
            hitGrid!!.addHit(laserEnd)
        }

//        val observed = getObservedObstaclesAndLandmarks(laserEnds, distances)
//        stroke(0f, 1f, 1f)
//        val obstacles = observed.first
//        for (segment in obstacles) {
//            line(segment.point1.a1.toFloat(), 0f, segment.point1.a2.toFloat(),
//                    segment.point2.a1.toFloat(), 0f, segment.point2.a2.toFloat())
//        }
//        val landmarks = observed.second
//        for (landmark in landmarks) {
//            circleXZ(landmark.a1, landmark.a2, 2.0)
//        }

        /* Draw */
        sim!!.draw()
        hitGrid!!.draw()

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}")
    }

    override fun keyPressed() {
        if (key == 'r') {
            reset()
        }
        if (key == 'p') {
            sim!!.togglePaused()
        }
        if (key == 'z') {
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
        if (key == 'x') {
            LaserSensor.DRAW_LASERS = !LaserSensor.DRAW_LASERS
        }
        if (key == 'l') {
            Simulator.DRAW_OBSTACLES = !Simulator.DRAW_OBSTACLES
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
