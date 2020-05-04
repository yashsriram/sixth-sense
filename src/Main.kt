import camera.QueasyCam
import extensions.*
import org.ejml.data.FMatrix2
import processing.core.PApplet
import processing.core.PConstants
import sensing.HitGrid
import sensing.IEP
import sensing.ObstacleLandmarkExtractor
import sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator
import java.util.*
import kotlin.math.roundToInt

class Main : PApplet() {
    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        var DRAW_OBSTACLES_LANDMARKS = true
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null
    private var hitGrid: HitGrid? = null
    private var extractor: ObstacleLandmarkExtractor? = null

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
        extractor = RANSACLeastSquares(this)
    }

    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        val poseCopy = sim!!.truePose // FIXME: should use estimated pose here later
        val position = FMatrix2(poseCopy.a1, poseCopy.a2)
        val orientation = poseCopy.a3
        val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
        centerToHead *= sim!!.robotRadius
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

        val (obstacles, landmarks) = extractor!!.getObservedObstaclesAndLandmarks(laserEnds, distances)

        if (DRAW_OBSTACLES_LANDMARKS) {
            stroke(1f, 0f, 1f)
            for (segment in obstacles) {
                line(segment.first.a1, 0f, segment.first.a2, segment.second.a1, 0f, segment.second.a2)
            }
            stroke(0f, 1f, 1f)
            for (landmark in landmarks) {
                circleXZ(landmark.a1, landmark.a2, 2f)
            }
        }

        /* Draw */
        sim!!.draw()
//        hitGrid!!.draw()

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}" +
                " extractor=${extractor!!.getName()}" +
                " #obs=${obstacles.size} #land=${landmarks.size}"
        )
    }

    override fun keyPressed() {
        if (key == '1') {
            extractor = RANSACLeastSquares(this)
            RANSACLeastSquares.USE_LEAST_SQUARE_FITTING = false
        }
        if (key == '2') {
            extractor = RANSACLeastSquares(this)
            RANSACLeastSquares.USE_LEAST_SQUARE_FITTING = true
        }
        if (key == '3') {
            extractor = IEP(this)
        }
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
        if (key == 'g') {
            Simulator.GHOST_MODE = !Simulator.GHOST_MODE
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
        if (key == 'f') {
            RANSACLeastSquares.DRAW_PARTITIONS = !RANSACLeastSquares.DRAW_PARTITIONS
            IEP.DRAW_PARTITIONS = !IEP.DRAW_PARTITIONS
        }
        if (key == 'm') {
            DRAW_OBSTACLES_LANDMARKS = !DRAW_OBSTACLES_LANDMARKS
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
