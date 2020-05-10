package demos

import camera.QueasyCam
import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
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

class Simulation : PApplet() {
    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        const val UPDATE_THRESHOLD = 20
        const val AUGMENT_THRESHOLD = 200
        const val PERIODICAL_CLEAN_EVERY_N_AUGMENT_UPDATES = 25
        const val PERIODICAL_CLEAN_THRESHOLD = 3
        const val ORIENTATION_SLACK = 0.01f
        const val MILESTONE_SLACK = 1f
        var DRAW_OBSTACLES_LANDMARKS = true
        var DRAW_ESTIMATED_LASERS = false
        var DRAW_ESTIMATED_PATH = true
        var DRAW_PLANNED_PATH = true
        var DRAW_TRUE_PATH = true
        var DRAW_BRESENHAM_POINTS = false
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null
    private val truePath = mutableListOf<FMatrix2>()
    private val estimatedPath = mutableListOf<FMatrix2>()

    // State estimate
    private var x_T = FMatrixRMaj()
    private var sigma_T = FMatrixRMaj()
    private val slam = SLAM()

    // Propagation covariance
    private val std_N = 0.10f
    private val sigma_N = CommonOps_FDRM.identity(2) * (std_N * std_N)

    // Measurement covariance
    private val std_M = 1f
    private val sigma_M = CommonOps_FDRM.identity(2) * (std_M * std_M)

    // Obstacle and landmark extractor
    private var extractor: ObstacleLandmarkExtractor? = null

    // Control
    private val control = FMatrix2(0f, 0f)

    // Propagated/Measured until
    private var propagatedUntil = 0f
    private var lastMeasured = 0L

    // Planning
    private var hitGrid: HitGrid? = null
    private var plannedCells = mutableListOf<Int>()
    private var plannedPath = mutableListOf<FMatrix2>()
    private val goal = FMatrix2()
    private var currentMilestone = 0

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
        // Start simulator
        val sceneName = args[0]
        sim = Simulator(this, sceneName)
        val initialTruePose = sim!!.getTruePose()
        // Init estimates with zero uncertainty
        x_T = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(initialTruePose.a1),
                        floatArrayOf(initialTruePose.a2),
                        floatArrayOf(initialTruePose.a3)
                )
        )
        sigma_T = CommonOps_FDRM.identity(3) * 0f
        // Reset slam
        slam.reset()
        // Init obstacle and landmark extractor
        extractor = RANSACLeastSquares(this)
        // Keep track of the path
        truePath.clear()
        estimatedPath.clear()
        truePath.add(FMatrix2(initialTruePose.a1, initialTruePose.a2))
        estimatedPath.add(FMatrix2(x_T[0], x_T[1]))
        // Start the robot
        propagatedUntil = 0f
        lastMeasured = System.currentTimeMillis()
        sim!!.applyControl(control)
        // Planning
        goal.set(x_T[0] + parseFloat(args[1]), x_T[1] + parseFloat(args[2]))
        hitGrid = HitGrid(FMatrix2(-1000f, -1000f), FMatrix2(1000f, 1000f), 500, 500)
        plannedCells = hitGrid!!.aStar(FMatrix2(x_T[0], x_T[1]), goal)
        plannedPath = hitGrid!!.coordinatesOf(plannedCells)
        currentMilestone = 0
    }

    private fun rePlan() {
        var replan = false
        for (cell in plannedCells) {
            if (hitGrid!!.hitsAt[cell] > 0) {
                replan = true
                break
            }
        }
        if (replan) {
            plannedCells = hitGrid!!.aStar(FMatrix2(x_T[0], x_T[1]), goal)
            plannedPath = hitGrid!!.coordinatesOf(plannedCells)
            currentMilestone = 0
        }
    }

    private fun updateControl() {
        if (currentMilestone < plannedPath.size - 1) {
            // Pull towards next milestone
            val toGoal = plannedPath[currentMilestone + 1] - FMatrix2(x_T[0], x_T[1])
            val goalOrientation = atan2(toGoal.a2, toGoal.a1)
            val toOrientation = goalOrientation - x_T[2]
            // Orient towards goal
            if (abs(toOrientation) > ORIENTATION_SLACK) {
                if (toOrientation > 0) {
                    control.set(0f, 0.1f)
                } else {
                    control.set(0f, -0.1f)
                }
                sim!!.applyControl(control)
                return
            }
            // Reached next milestone
            if (toGoal.norm() < MILESTONE_SLACK) {
                currentMilestone++
                return
            }
            // Move towards next milestone
            control.set(4f, 0f)
            sim!!.applyControl(control)
        } else {
            control.set(0f, 0f)
            sim!!.applyControl(control)
        }
    }
    fun swap(a:Any, b: Any): Pair<Any, Any> {
//        a = a + b
//        b = a - b
//        a = a - b
        return Pair(b,a)
    }

    fun bresenham2(x1:Int, y1:Int, x2:Int, y2:Int, sensedPts:MutableList<FMatrix2>)
    {
        if(PApplet.abs(y2 - y1) > PApplet.abs(x2 - x1))
        { //If the line is steep then it would be converted to non steep by changing coordinate
            val(x1,y1) = Pair(y1,x1)
            val(x2,y2) = Pair(y2,x2)
        }
        if(x1 >x2) {
            val (x1, x2) =  Pair(x2,x1)
            val (y1, y2) =  Pair(y2,y1)
        }
        var dx = PApplet.abs(x2 - x1)                              // Distance to travel in x-direction
        var dy = PApplet.abs(y2 - y1)                               //Distance to travel in y-direction
        var sx = sign(x2 - x1.toFloat())                                    //sx indicates direction of travel in X-dir
        var sy = sign(y2.toFloat() - y1.toFloat())                                     //Ensures positive slope line
        var x = x1
        var y = y1
        var param = 2*dy - dx
        val stepx = hitGrid!!.cellSizeX.toInt()
        val stepy = hitGrid!!.cellSizeY.toInt()

        for (i in 0 until ((dx - 1)/stepx).toInt()) {
            sensedPts.add(FMatrix2(x.toFloat(), y.toFloat()))
            param = param + 2 * dy                                     //parameter value is modified
            if (param > 0) {                                          //if parameter value is exceeded
                y = y + (stepy * sy).toInt()                                  //then y coordinate is increased
                param = param - 2 * (dx)                             //and parameter value is decreased
            }
            x = x + (stepx*sx).toInt()
        }
    }
    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        // Get time elapsed
        val latestTimeElapsed = sim!!.getTimeElapsed()
        val dt = latestTimeElapsed - propagatedUntil
        propagatedUntil = latestTimeElapsed

        // Run an EKFSLAMPropagation step
        val (x_TPDT, sigma_TPDT) = slam.propagateEKFSLAM(x_T, sigma_T, control, sigma_N, dt)
        x_T = x_TPDT
        sigma_T = sigma_TPDT

        // Re plan if path has obstacle in it
        rePlan()
        // Update control based on plan
        updateControl()
        val (distances, timestamp) = sim!!.getLaserMeasurement()
        if (timestamp > lastMeasured) {
            // Get the estimate of laser source position
            val position = FMatrix2(x_T[0], x_T[1])
            val orientation = x_T[2]
            val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
            centerToHead *= sim!!.getRobotRadius()
            val tail = position - centerToHead
            // Get the estimate of laser ends
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
                hitGrid!!.addHit(laserEnd, sim!!.getRobotRadius())
            }
            if (DRAW_ESTIMATED_LASERS) {
                noFill()
                for (laserEnd in laserEnds) {
                    stroke(0f, 0f, 1f)
                    line(tail.a1, 0f, tail.a2, laserEnd.a1, 0f, laserEnd.a2)
                    stroke(1f, 0f, 0f)
                    circleXZ(laserEnd.a1, laserEnd.a2, 1f)
                }
            }
            // Extract obstacles and landmarks
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

            // If > 0 landmarks detected
            // Run an EKFSLAMAugmentUpdate step
            val rel_pos_msmts = mutableListOf<FMatrixRMaj>()
            val sigma_Ms = mutableListOf<FMatrixRMaj>()
            if (landmarks.isNotEmpty()) {
                for (G_P_L in landmarks) {
                    val G_P_L__G_P_R_FMat2 = G_P_L - position
                    val G_P_L__G_P_R = FMatrixRMaj(
                            arrayOf(
                                    floatArrayOf(G_P_L__G_P_R_FMat2.a1),
                                    floatArrayOf(G_P_L__G_P_R_FMat2.a2)
                            )
                    )
                    val sinTheta = sin(orientation)
                    val cosTheta = cos(orientation)
                    val C_T = FMatrixRMaj(
                            arrayOf(
                                    floatArrayOf(cosTheta, sinTheta),
                                    floatArrayOf(-sinTheta, cosTheta)
                            )
                    )
                    val R_P_L = C_T * G_P_L__G_P_R
                    rel_pos_msmts.add(R_P_L)
                    sigma_Ms.add(sigma_M)
                }
            }
            val (x_Plus, sigma_Plus) = slam.augmentUpdateRelPosEKFSLAM(x_T, sigma_T, rel_pos_msmts, sigma_Ms)
            x_T = x_Plus
            sigma_T = sigma_Plus
            // Update last measured
            lastMeasured = timestamp
        }
        //BresenHam: Drawing
        if (DRAW_BRESENHAM_POINTS) {
            val sensedPts = mutableListOf<FMatrix2>()
            val tail = FMatrix2(0.0F, 0.0F)
            bresenham2(tail.a1.toInt(), tail.a2.toInt(),  900, 500, sensedPts)
            stroke(0f, 0f, 1f)
            line(tail.a1, 0f, tail.a2, tail.a1+ 900F, 0f, tail.a2+500F)
            stroke(1f, 0.5f, 0.5f)
            for (pts in sensedPts) {
                hitGrid!!.addHit(pts, sim!!.getRobotRadius())
                circleXZ(pts.a1, pts.a2, 2f)
            }
        }
        // Keep track of path
        val truePose = sim!!.getTruePose()
        truePath.add(FMatrix2(truePose.a1, truePose.a2))
        estimatedPath.add(FMatrix2(x_T[0], x_T[1]))

        /* Draw */
        sim!!.draw()
        hitGrid!!.draw(this)
        if (DRAW_PLANNED_PATH) {
            noFill()
            stroke(0f, 1f, 1f)
            pathXZ(plannedPath)
            if (currentMilestone < plannedPath.size - 1) {
                circleXZ(plannedPath[currentMilestone + 1].a1, plannedPath[currentMilestone + 1].a2, 5f)
            }
        }
        if (DRAW_TRUE_PATH) {
            stroke(0f, 1f, 0f)
            pathXZ(truePath)
        }
        if (DRAW_ESTIMATED_PATH) {
            // Draw the estimated trajectory
            stroke(0f, 0f, 1f)
            noFill()
            pathXZ(estimatedPath)
            circleXZ(estimatedPath.last().a1, estimatedPath.last().a2, sim!!.getRobotRadius())
            val position = FMatrix2(x_T[0], x_T[1])
            val orientation = x_T[2]
            val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
            centerToHead *= sim!!.getRobotRadius()
            val head = position + centerToHead
            line(x_T[0], 0f, x_T[1], head.a1, 0f, head.a2)
        }
        // Draw the uncertainty of the robot
        covarianceXZ(x_T[0, 0, 2, 1], sigma_T[0, 0, 2, 2])
        // Draw the uncertainty of all the landmarks in the state
        for (j in 3 until x_T.numRows step 2) {
            covarianceXZ(x_T[j, 0, 2, 1], sigma_T[j, j, 2, 2])
        }

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}"
                + " extractor=${extractor!!.getName()}"
                + " #landmarks=${(x_T.numRows - 3) / 2}"
        )
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
        if (key == 'c') {
            cam!!.controllable = !cam!!.controllable
        }
        if (key == 'z') {
            DRAW_ESTIMATED_LASERS = !DRAW_ESTIMATED_LASERS
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
        if (key == 'y') {
            DRAW_PLANNED_PATH = !DRAW_PLANNED_PATH
        }
        if (key == 'u') {
            DRAW_ESTIMATED_PATH = !DRAW_ESTIMATED_PATH
        }
        if (key == 'i') {
            DRAW_TRUE_PATH = !DRAW_TRUE_PATH
            Simulator.DRAW_ROBOT = !Simulator.DRAW_ROBOT
        }
    }
}
