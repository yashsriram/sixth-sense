import camera.QueasyCam
import extensions.circleXZ
import extensions.minus
import extensions.plus
import extensions.timesAssign
import org.ejml.data.FMatrix2
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
    private var extractionLogic: LandmarkObstacleExtractionLogicIEP? = null

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
        hitGrid = HitGrid(this, FMatrix2(-1000f, -1000f), FMatrix2(1000f, 1000f), 1000, 1000)
        extractionLogic = LandmarkObstacleExtractionLogicIEP(this)
    }

    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        val poseCopy = sim!!.truePose // FIXME: should use estimated pose here later
        val position = FMatrix2(poseCopy.a1, poseCopy.a2)
        val orientation = poseCopy.a3
        val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
        centerToHead *= 0.5f * sim!!.robotLength
        val tail = position - centerToHead

        val distances = sim!!.laserDistances
        val laserEnds: MutableList<FMatrix2> = ArrayList(LaserSensor.COUNT)
        for (i in distances.indices) {
            if (distances[i] == LaserSensor.INVALID_DISTANCE) {
                continue
            }
            val percentage = i / (LaserSensor.COUNT - 1f)
            val theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage
            val laserBeam = FMatrix2(kotlin.math.cos(orientation + theta), kotlin.math.sin(orientation + theta))
            laserBeam *= distances[i]
            val laserEnd = tail + laserBeam
            laserEnds.add(laserEnd)
//            hitGrid!!.addHit(laserEnd)
        }
//        print("Max hits: " + hitGrid!!.maxCount + "\r")
//        noFill()
//        for (laserEnd in laserEnds) {
//            stroke(1f, 0f, 1f)
//            line(tail.a1, 0f, tail.a2, laserEnd.a1, 0f, laserEnd.a2)
//            stroke(1f, 1f, 1f)
//            circleXZ(laserEnd.a1, laserEnd.a2, 1f)
//        }

        val observed = extractionLogic!!.getObservedObstaclesAndLandmarks(laserEnds, distances)
        print("# obstacles: ${observed.first.size} # landmarks: ${observed.second.size}\r")

        val obstacles = observed.first
        stroke(1f, 0f, 1f)
        for (segment in obstacles) {
            line(segment.first.a1, 0f, segment.first.a2, segment.second.a1, 0f, segment.second.a2)
        }
        val landmarks = observed.second
        stroke(0f, 1f, 1f)
        for (landmark in landmarks) {
            circleXZ(landmark.a1, landmark.a2, 2f)
        }

        /* Draw */
        sim!!.draw()
//        hitGrid!!.draw()

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
            sim!!.applyControl(FMatrix2())
        }
        if (keyCode == PConstants.UP) {
            sim!!.applyControl(FMatrix2(100f, 0f))
        }
        if (keyCode == PConstants.DOWN) {
            sim!!.applyControl(FMatrix2(-100f, 0f))
        }
        if (keyCode == PConstants.LEFT) {
            sim!!.applyControl(FMatrix2(0f, -1f))
        }
        if (keyCode == PConstants.RIGHT) {
            sim!!.applyControl(FMatrix2(0f, 1f))
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
        if (key == 'v') {
            HitGrid.DRAW = !HitGrid.DRAW
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
