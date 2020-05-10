package scratch

import camera.QueasyCam
import extensions.circleXZ
import extensions.minus
import extensions.plus
import extensions.timesAssign
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix3
import processing.core.PApplet
import processing.core.PConstants
import robot.planning.HitGrid
import robot.sensing.IEP
import robot.sensing.ObstacleLandmarkExtractor
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator
import java.util.*
import kotlin.math.roundToInt
import kotlin.math.sign

class Main : PApplet() {
    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        var DRAW_OBSTACLES_LANDMARKS = true
        var DRAW_SENSED_POINTS = true
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null
    private var hitGrid: HitGrid? = null
    private var sensedPts = mutableListOf<FMatrix2>()
    private var extractor: ObstacleLandmarkExtractor? = null
    var prePoseCopy:FMatrix3 = FMatrix3(0.0F, 0.0F, 0.0F)

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
        val sceneName = "data/simple_rectangle.scn"
        sim = Simulator(this, sceneName)
//        hitGrid = HitGrid(this, FMatrix2(-1000f, -1000f), FMatrix2(1000f, 1000f), 1000, 1000)
        extractor = RANSACLeastSquares(this)
        Simulator.GHOST_MODE = true
    }

    private fun swap(a:Any, b: Any): Pair<Any, Any> {
        var x = a
        var y = b
        return Pair(y,x)
    }
    private fun bresenham2(st: FMatrix2, end:FMatrix2)
    {
        var x1 = st.a1.toInt()
        var y1 = st.a2.toInt()
        var x2 = end.a1.toInt()
        var y2 = end.a2.toInt()

        if(abs(y2-y1)>abs(x2-x1))
        { //If the line is steep then it would be converted to non steep by changing coordinate
            val(x1,y1) = swap(x1, y1)
            val(x2,y2) = swap(x2, y2)
        }
        if(x1 >x2) {
            val (x1, x2) =  swap(x1, x2)
            val (y1, y2) =  swap(y1,y2)
        }
        var dx = abs(x2 - x1)                              // Distance to travel in x-direction
        var dy = abs(y2 - y1)                               //Distance to travel in y-direction
        var sx = sign(x2.toFloat() - x1.toFloat())                                    //sx indicates direction of travel in X-dir
        var sy = sign(y2.toFloat() - y1.toFloat())                                     //Ensures positive slope line
        var x = x1
        var y = y1
        var param = 2*dy - dx


        for (i in 0 until (dx - 1)/10) {
            sensedPts.add(FMatrix2(x.toFloat(), y.toFloat()))
            param = param + 2 * dy                                     //parameter value is modified
            if (param > 0) {                                          //if parameter value is exceeded
                var temp = y + 10 * sy                                      //then y coordinate is increased
                y = temp.toInt()
                param = param - 2 * (dx)                             //and parameter value is decreased
            }
            var temp = x + 10*sx
            x = temp.toInt()

//                print(x, y)
//                print("\n")
        }
    }

    private fun bresenham(st: FMatrix2, end:FMatrix2) {
        val x1 = st.a1.toInt()
        val y1 = st.a2.toInt()
        val x2 = end.a1.toInt()
        val y2 = end.a2.toInt()
        val dy = (y2 - y1)
        val dx = (x2 - x1)
        val mNew = 2 * dy
        var slopeErrorNew = mNew - dx
        var x = x1
        var y = y1
        while (x <= x2) {

            sensedPts.add(FMatrix2(x.toFloat(), y.toFloat()))
            if(slopeErrorNew>=0){
                y = (y.toInt()) + 50
                slopeErrorNew += mNew -2*dx
            }
            else
            {
                slopeErrorNew += mNew
                x = x.toInt() + 50
            }
//            print(x, y)
//            print("\n")
        }
    }
    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        val poseCopy = sim!!.getTruePose() // FIXME: should use estimated pose here later
        val position = FMatrix2(poseCopy.a1, poseCopy.a2)
        val orientation = poseCopy.a3
        val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
        centerToHead *= sim!!.getRobotRadius()
        val tail = position - centerToHead

        val (distances, timestamp) = sim!!.getLaserMeasurement()
        val laserEnds: MutableList<FMatrix2> = ArrayList(LaserSensor.COUNT)
        val sensedEnds: MutableList<FMatrix2> = ArrayList(LaserSensor.COUNT)
        for (i in distances.indices) {

            val percentage = i / (LaserSensor.COUNT - 1f)
            val theta = LaserSensor.MIN_THETA + (LaserSensor.MAX_THETA - LaserSensor.MIN_THETA) * percentage
            val laserBeam = FMatrix2(kotlin.math.cos(orientation + theta), kotlin.math.sin(orientation + theta))
            laserBeam *= distances[i]
            val laserEnd = tail + laserBeam
            sensedEnds.add(laserEnd)
            if (distances[i] != LaserSensor.INVALID_DISTANCE) {
                laserEnds.add(laserEnd)
//              hitGrid!!.addHit(laserEnd)
            }

//            print(sensedPts[0])
        }

//        if(prePoseCopy -poseCopy != FMatrix3(0.0F, 0.0F, 0.0F)){
//        for(i in  0 until 15)
//        {
//            bresenham2(tail, sensedEnds[i])
//        }
//        }
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

        if (DRAW_SENSED_POINTS) {
            stroke(1f, 0.5f, 0.5f)
            for (pt in sensedPts) {
                circleXZ(pt.a1, pt.a2, 2f)
            }
        }
        /* Draw */
        sim!!.draw()
//        hitGrid!!.draw()

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}" +
                " extractor=${extractor!!.getName()}" +
                " #obs=${obstacles.size} #land=${landmarks.size}"
        )
        prePoseCopy = poseCopy
    }

    override fun keyPressed() {
        if (key == '1') {
            extractor = IEP(this)
        }
        if (key == '2') {
            extractor = RANSACLeastSquares(this)
            RANSACLeastSquares.USE_LEAST_SQUARE_FITTING = false
        }
        if (key == '3') {
            extractor = RANSACLeastSquares(this)
            RANSACLeastSquares.USE_LEAST_SQUARE_FITTING = true
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
    val appletArgs = arrayOf("scratch.Main")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
