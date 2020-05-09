import camera.QueasyCam
import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
import processing.core.PApplet
import processing.core.PConstants
import robot.calibaration.RK4Integrator
import robot.sensing.HitGrid
import robot.sensing.IEP
import robot.sensing.ObstacleLandmarkExtractor
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator
import java.util.*
import kotlin.math.roundToInt

class SLAM : PApplet() {
    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        var DRAW_OBSTACLES_LANDMARKS = true
        var DRAW_ESTIMATED_LASERS = true
    }

    private var sim: Simulator? = null
    private var cam: QueasyCam? = null
    private val truePath = mutableListOf<FMatrix2>()
    private val estimatedPath = mutableListOf<FMatrix2>()

    // State estimate
    private var x_T = FMatrixRMaj()
    private var sigma_T = FMatrixRMaj()

    // Propagation covariance
    private val std_N = 0.10f
    private val sigma_N = CommonOps_FDRM.identity(2) * (std_N * std_N)

    // Obstacle and landmark extractor
    private var extractor: ObstacleLandmarkExtractor? = null

    // Control
    private val u = FMatrix2(4f, 0.15f)

    // Propagated/Measured until
    private var propagatedUntil = 0f
    private var lastMeasured = 0L

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
        Simulator.GHOST_MODE = true
        // Start simulator
        val sceneName = "data/simple_rectangle.scn"
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
        // Init obstacle and landmark extractor
        extractor = RANSACLeastSquares(this)
        // Starts the robot
        propagatedUntil = 0f
        lastMeasured = System.currentTimeMillis()
        sim!!.applyControl(u)
    }

    private fun propagateEKFSLAM(x_T: FMatrixRMaj,
                                 sigma_T: FMatrixRMaj,
                                 u: FMatrix2,
                                 sigma_N: FMatrixRMaj,
                                 dt: Float): Pair<FMatrixRMaj, FMatrixRMaj> {
        val x_TPDT = RK4Integrator.updatePose(x_T, u, dt, 1)

        val sigma_TPDT = FMatrixRMaj(sigma_T)
        val theta_t = x_T[2]
        val v = u.a1
        val A = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(1f, 0f, -dt * v * sin(theta_t)),
                        floatArrayOf(0f, 1f, dt * v * cos(theta_t)),
                        floatArrayOf(0f, 0f, 1f)
                )
        )
        val N = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(dt * cos(theta_t), 0f),
                        floatArrayOf(dt * sin(theta_t), 0f),
                        floatArrayOf(0f, dt)
                )
        )
        // Sigma_RR_new
        val Sigma_RR = sigma_TPDT[0, 0, 3, 3]
        sigma_TPDT[0, 0, 3, 3] = A * Sigma_RR * A.transpose() + N * sigma_N * N.transpose()
        // Sigma_R_Li_new
        for (col in 3 until sigma_TPDT.numCols step 2) {
            sigma_TPDT[0, col, 3, 2] = A * sigma_TPDT[0, col, 3, 2]
        }
        // Sigma_Li_R_new
        for (row in 3 until sigma_TPDT.numRows step 2) {
            sigma_TPDT[row, 0, 2, 3] = sigma_TPDT[row, 0, 2, 3] * A.transpose()
        }
        return Pair(x_TPDT, sigma_TPDT)
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
        val (x_TPDT, sigma_TPDT) = propagateEKFSLAM(x_T, sigma_T, u, sigma_N, dt)
        x_T = x_TPDT
        sigma_T = sigma_TPDT

        // Keep track of path
        val truePose = sim!!.getTruePose()
        truePath.add(FMatrix2(truePose.a1, truePose.a2))
        estimatedPath.add(FMatrix2(x_T[0], x_T[1]))

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
            }
            if (DRAW_ESTIMATED_LASERS) {
                noFill()
                for (laserEnd in laserEnds) {
                    stroke(1f, 1f, 0f)
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
            // Update last measured
            lastMeasured = timestamp
        }

        /* Draw */
        sim!!.draw()
        // Draw the true trajectory
        stroke(0f, 1f, 0f)
        pathXZ(truePath)
        // Draw the estimated trajectory
        stroke(0f, 0f, 1f)
        pathXZ(estimatedPath)
        circleXZ(estimatedPath.last().a1, estimatedPath.last().a2, sim!!.getRobotRadius())
        // Draw the uncertainty of the robot
        covarianceXZ(x_T[0, 0, 2, 1], sigma_T[0, 0, 2, 2])

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}"
                + " extractor=${extractor!!.getName()}"
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
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("SLAM")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
