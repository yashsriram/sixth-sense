import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.CommonOps_DDRM
import processing.core.PApplet
import processing.core.PConstants
import java.util.*
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.roundToLong

class Slam : PApplet() {
    companion object {
        const val WIDTH = 1000
        const val HEIGHT = 1000
        const val NUM_LANDMARKS = 40
    }

    private val landmarks = mutableListOf<DMatrixRMaj>()
    private val truePath = mutableListOf<DMatrixRMaj>()
    private val estimatedPath = mutableListOf<DMatrixRMaj>()

    // Source of randomness
    private val random = Random()

    // Init agent
    private val xTrue = DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(WIDTH / 2.0 - 100),
                    doubleArrayOf(HEIGHT / 2.0),
                    doubleArrayOf(-PI / 2.0)
            )
    )

    // FIXME: Initial estimate itself has some noise?
    private val std_X = 0.05
    private val x_T = xTrue + DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(std_X * random.nextGaussian()),
                    doubleArrayOf(std_X * random.nextGaussian()),
                    doubleArrayOf(std_X * random.nextGaussian())
            )
    )
    private val sigma_T = CommonOps_DDRM.identity(3) * (std_X * std_X)

    // Noise Covariance
    private val std_N = 0.10
    private val sigma_N = CommonOps_DDRM.identity(2) * (std_N * std_N)
    private val std_M = 0.05
    private val sigma_M = CommonOps_DDRM.identity(2) * (std_M * std_M)

    // Control
    private val u = DMatrix2(30.0, 0.2)

    // Time step
    private val dt = 0.1
    private var iter = 1

    override fun settings() {
        size(WIDTH, HEIGHT, PConstants.P2D)
    }

    override fun setup() {
        surface.setTitle("Processing")
        colorMode(PConstants.RGB, 1.0f)
        rectMode(PConstants.CENTER)
        noStroke()

        // Generate Point Landmarks:
        for (i in 1..NUM_LANDMARKS) {
            val landmark = DMatrixRMaj(
                    arrayOf(
                            doubleArrayOf(ThreadLocalRandom.current().nextDouble(0.0, WIDTH.toDouble())),
                            doubleArrayOf(ThreadLocalRandom.current().nextDouble(0.0, HEIGHT.toDouble()))
                    )
            )
            landmarks.add(landmark)
        }

        // Init path
        truePath.add(xTrue)
        estimatedPath.add(x_T)
    }

    override fun draw() {
        // Run
        iter++
        if (iter < 1000) {
            // True Propagation without approximation (we'll assume w is not close to 0):
            val xTrueNew = DMatrixRMaj(truePath[truePath.size - 1])
            val n = DMatrix2(std_N * random.nextGaussian(), std_N * random.nextGaussian())
            val uTmp = n + u
            val thetaOld = xTrueNew[2]
            val thetaNew = thetaOld + dt * uTmp.a2
            xTrueNew[0] += uTmp.a1 / uTmp.a2 * (sin(thetaNew.toFloat()) - sin(thetaOld.toFloat()))
            xTrueNew[1] += uTmp.a1 / uTmp.a2 * (-cos(thetaNew.toFloat()) + cos(thetaOld.toFloat()))
            xTrueNew[2] = thetaNew

            // Run an EKFSLAMPropagation step
//            {
//                Eigen::VectorXd x_new;
//                Eigen::MatrixXd Sigma_new;
//                EKFSLAMPropagate(x_t, Sigma_t, u, Sigma_n, dt, x_new, Sigma_new);
//                x_t = x_new;
//                Sigma_t = Sigma_new;
//            }

            // Run an update every 5 iterations
            if (iter % 5 == 0) {
                // Compute measurements to all the landmarks within vicinity
                val noisyMeasurements = mutableListOf<DMatrixRMaj>()
                val noisyMeasurementSigmas = mutableListOf<DMatrixRMaj>()
                for (lm in landmarks) {
                    val truePositionToLandmark = lm[0, 0, 2, 1] - xTrueNew[0, 0, 2, 1]
                    if (truePositionToLandmark.norm() < 50.0) {
                        val theta = xTrueNew[2].toFloat()
                        val sinTheta = sin(theta).toDouble()
                        val cosTheta = cos(theta).toDouble()
                        val C_T = DMatrixRMaj(
                                arrayOf(
                                        doubleArrayOf(cosTheta, sinTheta),
                                        doubleArrayOf(-sinTheta, cosTheta)
                                )
                        )
                        val measurement = C_T * truePositionToLandmark
                        val m = DMatrixRMaj(
                                arrayOf(
                                        doubleArrayOf(std_M * random.nextGaussian()),
                                        doubleArrayOf(std_M * random.nextGaussian())
                                )
                        )
                        val noisyMeasurement = measurement + m
                        noisyMeasurements.add(noisyMeasurement)
                        noisyMeasurementSigmas.add(sigma_M)
                    }
                }

                // Run an EKFSLAMUpdate step
                if (noisyMeasurements.size > 0) {
//                    Eigen::VectorXd x_new;
//                    Eigen::MatrixXd Sigma_new;
//                    EKFSLAMRelPosUpdate(x_t, Sigma_t, noisyMeasurements, noisyMeasurementSigmas, x_new, Sigma_new);
//                    x_t = x_new;
//                    Sigma_t = Sigma_new;
                }
            }

            truePath.add(xTrueNew)
            estimatedPath.add(x_T)
        }

        // Draw
        background(0)
        noFill()
        stroke(1f, 1f, 0f)
        // Draw true landmarks
        for (landmark in landmarks) {
            circle(landmark[0].toFloat(), landmark[1].toFloat(), 10f)
        }

        // Draw the true trajectory
        stroke(0f, 1f, 0f)
        for (i in 1 until truePath.size) {
            val prevState = truePath[i - 1]
            val currState = truePath[i]
            line(prevState[0].toFloat(), prevState[1].toFloat(), currState[0].toFloat(), currState[1].toFloat())
        }
        circle(truePath[truePath.size - 1][0].toFloat(), truePath[truePath.size - 1][1].toFloat(), 10f)

        // Draw the estimated trajectory
        stroke(0f, 0f, 1f)
        for (i in 1 until estimatedPath.size) {
            val prevState = estimatedPath[i - 1]
            val currState = estimatedPath[i]
            line(prevState[0].toFloat(), prevState[1].toFloat(), currState[0].toFloat(), currState[1].toFloat())
        }
        circle(estimatedPath[estimatedPath.size - 1][0].toFloat(), estimatedPath[estimatedPath.size - 1][1].toFloat(), 10f)
        // Draw the uncertainty of the robot
//        vis.AddTempEllipse(x_t.head < 2 > (), Sigma_t.topLeftCorner < 2, 2 > (), Color::RED, 1.2);

        // Draw the uncertainty of all the landmarks in the state
//        for (size_t j = 3; j < x_t.size(); j += 2) {
//            vis.AddTempEllipse(x_t.segment < 2 > (j), Sigma_t.block < 2, 2 > (j, j), Color::RED, 1.2);
//        }

        surface.setTitle("Processing - FPS: " + frameRate.roundToLong())
    }

}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("Slam")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
