import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.CommonOps_DDRM
import org.ejml.dense.row.EigenOps_DDRM
import org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_DDRM
import processing.core.PApplet
import processing.core.PConstants
import java.lang.IllegalStateException
import java.util.*
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.roundToLong

data class Estimate(val mean: DMatrixRMaj, val covariance: DMatrixRMaj)

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
    private var x_T = xTrue + DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(std_X * random.nextGaussian()),
                    doubleArrayOf(std_X * random.nextGaussian()),
                    doubleArrayOf(std_X * random.nextGaussian())
            )
    )
    private var sigma_T = CommonOps_DDRM.identity(3) * (std_X * std_X)

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

    private fun propagateEKFSLAM(x_T: DMatrixRMaj,
                                 sigma_T: DMatrixRMaj,
                                 u: DMatrix2,
                                 sigma_N: DMatrixRMaj,
                                 dt: Double): Estimate {
        val theta_t = x_T[2]
        val v = u.a1
        val w = u.a2

        // Note that these we passed by reference, so to return, just set them
        val x_TPDT = DMatrixRMaj(x_T)
        x_TPDT[0] += dt * v * cos(theta_t.toFloat())
        x_TPDT[1] += dt * v * sin(theta_t.toFloat())
        x_TPDT[2] += dt * w

        val sigma_TPDT = DMatrixRMaj(sigma_T)
        val A = DMatrixRMaj(
                arrayOf(
                        doubleArrayOf(1.0, 0.0, -dt * v * sin(theta_t.toFloat())),
                        doubleArrayOf(0.0, 1.0, dt * v * cos(theta_t.toFloat())),
                        doubleArrayOf(0.0, 0.0, 1.0)
                )
        )
        val N = DMatrixRMaj(
                arrayOf(
                        doubleArrayOf(dt * cos(theta_t.toFloat()), 0.0),
                        doubleArrayOf(dt * sin(theta_t.toFloat()), 0.0),
                        doubleArrayOf(0.0, dt)
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


    override fun draw() {
        /* ---- ---- ---- ---- Update ---- ---- ---- ---- */
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
            val estimateTPDT = propagateEKFSLAM(x_T, sigma_T, u, sigma_N, dt)
            x_T = estimateTPDT.mean
            sigma_T = estimateTPDT.covariance

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

        /* ---- ---- ---- ---- Draw ---- ---- ---- ---- */
        background(0)
        noFill()
        stroke(1f, 1f, 0f)
        // Draw true landmarks
        for (landmark in landmarks) {
            circle(landmark[0].toFloat(), landmark[1].toFloat(), 10f)
        }

        // Draw the true trajectory
        jointPoints(truePath, 0f, 1f, 0f)
        circle(truePath[truePath.size - 1][0].toFloat(), truePath[truePath.size - 1][1].toFloat(), 10f)

        // Draw the estimated trajectory
        jointPoints(estimatedPath, 0f, 0f, 1f)
        circle(estimatedPath[estimatedPath.size - 1][0].toFloat(), estimatedPath[estimatedPath.size - 1][1].toFloat(), 10f)

        // Draw the uncertainty of the robot
        visualizeCovariance(x_T[0, 0, 2, 1], sigma_T[0, 0, 2, 2])

        // Draw the uncertainty of all the landmarks in the state
        for (j in 3 until x_T.numRows step 2) {
            visualizeCovariance(x_T[j, 0, 2, 1], sigma_T[j, j, 2, 2])
        }

        surface.setTitle("Processing - FPS: " + frameRate.roundToLong())
    }

    private fun visualizeCovariance(x_T: DMatrixRMaj, sigma_T: DMatrixRMaj) {
        val sigma_T_copy = DMatrixRMaj(sigma_T)
        val decomposer = SwitchingEigenDecomposition_DDRM(2, true, 1e-6)
        val success = decomposer.decompose(sigma_T_copy)
        if (!success) {
            throw IllegalStateException("Can't find eigen vectors/values of matrix")
        }
        val eigenValue1 = decomposer.getEigenvalue(0)
        val eigenValue2 = decomposer.getEigenvalue(1)
        val eigenVectors = EigenOps_DDRM.createMatrixV(decomposer)
        val ellipseTheta = atan2(eigenVectors[1, 0].toFloat(), eigenVectors[0, 0].toFloat())
        val sinEllipseTheta = sin(ellipseTheta)
        val cosEllipseTheta = cos(ellipseTheta)
        val rot = DMatrixRMaj(
                arrayOf(
                        doubleArrayOf(cosEllipseTheta.toDouble(), -sinEllipseTheta.toDouble()),
                        doubleArrayOf(sinEllipseTheta.toDouble(), cosEllipseTheta.toDouble())
                )
        )
        val ellipseResolution = 20
        val ellipse = mutableListOf<DMatrixRMaj>()
        for (i in 0 until ellipseResolution) {
            // Only ellipseResolution - 1 points as the first and last points are the same (completing the loop)
            val theta = 2 * PI * i / (ellipseResolution - 1)

            // Scale by major / minor axis, then rotate and offset
            val pointOnEllipse = DMatrixRMaj(
                    arrayOf(
                            doubleArrayOf(2.0 * sqrt((5.991 * eigenValue1.real).toFloat()) * cos(theta)),
                            doubleArrayOf(2.0 * sqrt((5.991 * eigenValue2.real).toFloat()) * sin(theta))
                    )
            )
            ellipse.add(x_T + rot * pointOnEllipse)
        }
        jointPoints(ellipse, 1f, 0f, 0f)
    }

    private fun jointPoints(points: MutableList<DMatrixRMaj>, r: Float, g: Float, b: Float) {
        stroke(r, g, b)
        for (i in 1 until points.size) {
            val prevState = points[i - 1]
            val currState = points[i]
            line(prevState[0].toFloat(), prevState[1].toFloat(), currState[0].toFloat(), currState[1].toFloat())
        }
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
