import camera.QueasyCam
import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
import processing.core.PApplet
import processing.core.PConstants
import robot.sensing.HitGrid
import robot.sensing.IEP
import robot.sensing.ObstacleLandmarkExtractor
import robot.sensing.RANSACLeastSquares
import scratch.Estimate
import simulator.LaserSensor
import simulator.Simulator
import kotlin.math.roundToInt

class SLAM : PApplet() {
    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        var DRAW_OBSTACLES_LANDMARKS = true
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

    // Clock
    private var timePropagatedTo = 0f

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
        timePropagatedTo = 0f
        sim!!.applyControl(u)
    }

    private fun propagateEKFSLAM(x_T: FMatrixRMaj,
                                 sigma_T: FMatrixRMaj,
                                 u: FMatrix2,
                                 sigma_N: FMatrixRMaj,
                                 dt: Float): Estimate {
        val theta_t = x_T[2]
        val v = u.a1
        val w = u.a2

        val x_TPDT = FMatrixRMaj(x_T)
        x_TPDT[0] += dt * v * cos(theta_t)
        x_TPDT[1] += dt * v * sin(theta_t)
        x_TPDT[2] += dt * w

        val sigma_TPDT = FMatrixRMaj(sigma_T)
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
        return Estimate(mean = x_TPDT, covariance = sigma_TPDT)
    }

    var sumdt = 0f

    override fun draw() {
        /* Clear screen */
        background(0)

        /* Update */
        // Get time elapsed
        val latestTimeElapsed = sim!!.getTimeElapsed()
        val dt = latestTimeElapsed - timePropagatedTo
        timePropagatedTo = latestTimeElapsed

        // Run an EKFSLAMPropagation step
        val estimateTPDT = propagateEKFSLAM(x_T, sigma_T, u, sigma_N, dt)

        x_T = estimateTPDT.mean
        sigma_T = estimateTPDT.covariance

        val truePose = sim!!.getTruePose()
        truePath.add(FMatrix2(truePose.a1, truePose.a2))
        estimatedPath.add(FMatrix2(x_T[0], x_T[1]))

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