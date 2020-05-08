package scratch

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
import org.ejml.dense.row.EigenOps_FDRM
import org.ejml.dense.row.decomposition.eig.SwitchingEigenDecomposition_FDRM
import processing.core.PApplet
import processing.core.PConstants
import java.util.*
import java.util.concurrent.ThreadLocalRandom
import kotlin.Float.Companion.POSITIVE_INFINITY
import kotlin.math.roundToLong

data class Estimate(val mean: FMatrixRMaj, val covariance: FMatrixRMaj)

class HW2 : PApplet() {
    companion object {
        const val WIDTH = 1000
        const val HEIGHT = 1000
        const val NUM_LANDMARKS = 40
        const val SENSOR_DISTANCE = 200f
    }

    private val landmarks = mutableListOf<FMatrixRMaj>()
    private val truePath = mutableListOf<FMatrixRMaj>()
    private val estimatedPath = mutableListOf<FMatrixRMaj>()

    // Source of randomness
    private val random = Random()

    // Init agent
    private val xTrue = FMatrixRMaj(
            arrayOf(
                    floatArrayOf(WIDTH / 2f - 200),
                    floatArrayOf(HEIGHT / 2f),
                    floatArrayOf(-PI / 2f)
            )
    )

    // FIXME: Initial estimate itself has some noise?
    private val std_X = 0f
    private var x_T = xTrue + FMatrixRMaj(
            arrayOf(
                    floatArrayOf((std_X * random.nextGaussian()).toFloat()),
                    floatArrayOf((std_X * random.nextGaussian()).toFloat()),
                    floatArrayOf((std_X * random.nextGaussian()).toFloat())
            )
    )
    private var sigma_T = CommonOps_FDRM.identity(3) * (std_X * std_X)

    // Noise Covariance
    private val std_N = 0.10f
    private val sigma_N = CommonOps_FDRM.identity(2) * (std_N * std_N)
    private val std_M = 7f
    private val sigma_M = CommonOps_FDRM.identity(2) * (std_M * std_M)

    // Control
    private var u = FMatrix2(20f, 0.15f)

    // Time step
    private val dt = 0.05f
    private var iter = 1

    override fun settings() {
        size(WIDTH, HEIGHT, PConstants.P2D)
    }

    override fun setup() {
        surface.setTitle("Processing")
        colorMode(PConstants.RGB, 1f)
        rectMode(PConstants.CENTER)
        noStroke()

        // Generate Point Landmarks:
        for (i in 1..NUM_LANDMARKS) {
            val landmark = FMatrixRMaj(
                    arrayOf(
                            floatArrayOf(ThreadLocalRandom.current().nextDouble(0.0, WIDTH.toDouble()).toFloat()),
                            floatArrayOf(ThreadLocalRandom.current().nextDouble(0.0, HEIGHT.toDouble()).toFloat())
                    )
            )
            landmarks.add(landmark)
        }

        // Init path
        truePath.add(xTrue)
        estimatedPath.add(x_T)
    }

    private fun propagateEKFSLAM(x_T: FMatrixRMaj,
                                 sigma_T: FMatrixRMaj,
                                 u: FMatrix2,
                                 sigma_N: FMatrixRMaj,
                                 dt: Float): Estimate {
        val theta_t = x_T[2]
        val v = u.a1
        val w = u.a2

        // Note that these we passed by reference, so to return, just set them
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

    private fun updateRelPosEKFSLAM(x_hat_t: FMatrixRMaj,
                            Sigma_x_t: FMatrixRMaj,
                            zs: MutableList<FMatrixRMaj>,
                            Sigma_ms: MutableList<FMatrixRMaj>): Estimate {
        // For each measurement, check if it matches any already in the state, and run an update for it.
        // For every unmatched measurement make sure it's sufficiently novel, then add to the state.
        var x_TPDT = FMatrixRMaj(x_hat_t);
        var sigma_TPDT = FMatrixRMaj(Sigma_x_t);
        for (i in 0 until zs.size) {
            // For each measurement
            val z = zs[i]
            val Sigma_m = Sigma_ms[i]

            // Best match quantities
            var min_distance = POSITIVE_INFINITY
            var best_H = FMatrixRMaj()
            var best_h_x_hat_0 = FMatrixRMaj()
            var best_S = FMatrixRMaj()

            // For each landmark check the Mahalanobis distance
            for (j in 3 until x_TPDT.numRows step 2) {
                val x_R_T = x_TPDT[0, 0]
                val y_R_T = x_TPDT[1, 0]
                val theta_T = x_TPDT[2, 0]
                val G_p_R = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(x_R_T),
                                floatArrayOf(y_R_T)
                        )
                )
                val sinTheta_T = sin(theta_T)
                val cosTheta_T = cos(theta_T)
                val H_L_new = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(cosTheta_T, sinTheta_T),
                                floatArrayOf(-sinTheta_T, cosTheta_T)
                        )
                )
                val x_L_T = x_TPDT[j, 0]
                val y_L_t = x_TPDT[j + 1, 0]
                val G_p_L = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(x_L_T),
                                floatArrayOf(y_L_t)
                        )
                )
                val H_R = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(-cosTheta_T, -sinTheta_T, -sinTheta_T * (x_L_T - x_R_T) + cosTheta_T * (y_L_t - y_R_T)),
                                floatArrayOf(sinTheta_T, -cosTheta_T, -cosTheta_T * (x_L_T - x_R_T) - sinTheta_T * (y_L_t - y_R_T))
                        )
                )

                val H = FMatrixRMaj(2, x_TPDT.numRows)
                H[0, 0, 2, 3] = H_R
                H[0, j, 2, 2] = H_L_new

                val S = H * sigma_TPDT * H.transpose() + Sigma_m
                val h_x_hat_0 = H_L_new * (G_p_L - G_p_R)
                val residue = z - h_x_hat_0
                val distance = residue.transpose() * S.inverse() * residue

                assert(distance.numElements == 1)

                // Track the most likely landmark
                if (distance[0, 0] < min_distance) {
                    min_distance = distance[0, 0]
                    best_H = H
                    best_h_x_hat_0 = h_x_hat_0
                    best_S = S
                }
            }

            // If looks like a landmark then do a regular update
            if (min_distance <= 20) {
                val K = sigma_TPDT * best_H.transpose() * best_S.inverse()
                val I = CommonOps_FDRM.identity(x_TPDT.numRows)

                // Note that these we passed by reference, so to return, just set them
                x_TPDT.plusAssign(K * (z - best_h_x_hat_0))
                val term = I - K * best_H
                sigma_TPDT = term * sigma_TPDT * term.transpose() + K * Sigma_m * K.transpose()
                continue
            }

            // If looks like no landmark seen until now augment SLAM state with the landmark information
            if (min_distance > 25) {
                val x_R_T = x_TPDT[0, 0]
                val y_R_T = x_TPDT[1, 0]
                val theta_T = x_TPDT[2, 0]
                val G_p_R = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(x_R_T),
                                floatArrayOf(y_R_T)
                        )
                )
                val sinTheta_T = sin(theta_T)
                val cosTheta_T = cos(theta_T)
                val H_L_new = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(cosTheta_T, sinTheta_T),
                                floatArrayOf(-sinTheta_T, cosTheta_T)
                        )
                )

                // Expected value
                // Copy previous state
                val prevX = x_TPDT
                x_TPDT = FMatrixRMaj(x_TPDT.numRows + 2, 1)
                for (t in 0 until prevX.numRows) {
                    x_TPDT[t, 0] = prevX[t, 0]
                }
                // Add new landmark estimate
                val h_x_hat_0 = H_L_new * (FMatrixRMaj(2, 1) - G_p_R)
                x_TPDT[prevX.numRows, 0, 2, 1] = H_L_new.inverse() * (z - h_x_hat_0)

                // Covariance
                // Copy previous state
                val prevSigma = sigma_TPDT
                sigma_TPDT = FMatrixRMaj(sigma_TPDT.numRows + 2, sigma_TPDT.numCols + 2)
                for (t in 0 until prevSigma.numRows) {
                    for (s in 0 until prevSigma.numCols) {
                        sigma_TPDT[t, s] = prevSigma[t, s]
                    }
                }
                // Augmentation
                val H_R = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(-cosTheta_T, -sinTheta_T, -(0 - x_R_T) * sinTheta_T + (0 - y_R_T) * cosTheta_T),
                                floatArrayOf(sinTheta_T, -cosTheta_T, -(0 - x_R_T) * cosTheta_T - (0 - y_R_T) * sinTheta_T)
                        )
                )
                val H_L_new_inv = H_L_new.inverse()
                val top3x3 = prevSigma[0, 0, 3, 3]
                // Top right block
                sigma_TPDT[0, prevSigma.numCols, 3, 2] =
                        top3x3 * H_R.transpose() * H_L_new_inv.transpose() * -1f
                // Bottom left block
                sigma_TPDT[prevSigma.numRows, 0, 2, 3] =
                        H_L_new_inv * H_R * top3x3 * -1f
                // Bottom right block
                sigma_TPDT[prevSigma.numRows, prevSigma.numCols, 2, 2] =
                        H_L_new_inv * (H_R * top3x3 * H_R.transpose() + Sigma_m) * H_L_new_inv.transpose()
                // Bottom row
                for (col in 3 until prevSigma.numCols step 2) {
                    sigma_TPDT[prevSigma.numRows, col, 2, 2] =
                            H_L_new_inv * H_R * prevSigma[0, col, 3, 2] * -1f
                }
                // Right row
                for (row in 3 until prevSigma.numRows step 2) {
                    sigma_TPDT[row, prevSigma.numCols, 2, 2] =
                            prevSigma[row, 0, 2, 3] * H_R.transpose() * H_L_new_inv.transpose() * -1f
                }
                continue
            }
        }

        return Estimate(mean = x_TPDT, covariance = sigma_TPDT)
    }

    override fun draw() {
        /* ---- ---- ---- ---- Update ---- ---- ---- ---- */
        iter++
        // True Propagation without approximation (we'll assume w is not close to 0):
        val xTrueNew = FMatrixRMaj(truePath[truePath.size - 1])
        val n = FMatrix2((std_N * random.nextGaussian()).toFloat(), (std_N * random.nextGaussian()).toFloat())
        val uTmp = n + u
        val thetaOld = xTrueNew[2]
        val thetaNew = thetaOld + dt * uTmp.a2
        xTrueNew[0] += uTmp.a1 / uTmp.a2 * (sin(thetaNew) - sin(thetaOld))
        xTrueNew[1] += uTmp.a1 / uTmp.a2 * (-cos(thetaNew) + cos(thetaOld))
        xTrueNew[2] = thetaNew

        // Run an EKFSLAMPropagation step
        val estimateTPDT = propagateEKFSLAM(x_T, sigma_T, u, sigma_N, dt)
        x_T = estimateTPDT.mean
        sigma_T = estimateTPDT.covariance

        // Run an update every 5 iterations
        val lasers = mutableListOf<FMatrixRMaj>()
        if (iter % 5 == 0) {
            // Compute measurements to all the landmarks within vicinity
            val noisyMeasurements = mutableListOf<FMatrixRMaj>()
            val noisyMeasurementSigmas = mutableListOf<FMatrixRMaj>()
            for (lm in landmarks) {
                val truePositionToLandmark = lm[0, 0, 2, 1] - xTrueNew[0, 0, 2, 1]
                if (truePositionToLandmark.norm() < SENSOR_DISTANCE) {
                    lasers.add(FMatrixRMaj(lm))
                    val theta = xTrueNew[2]
                    val sinTheta = sin(theta)
                    val cosTheta = cos(theta)
                    val C_T = FMatrixRMaj(
                            arrayOf(
                                    floatArrayOf(cosTheta, sinTheta),
                                    floatArrayOf(-sinTheta, cosTheta)
                            )
                    )
                    val measurement = C_T * truePositionToLandmark
                    val m = FMatrixRMaj(
                            arrayOf(
                                    floatArrayOf((std_M * random.nextGaussian()).toFloat()),
                                    floatArrayOf((std_M * random.nextGaussian()).toFloat())
                            )
                    )
                    val noisyMeasurement = measurement + m
                    noisyMeasurements.add(noisyMeasurement)
                    noisyMeasurementSigmas.add(sigma_M)
                }
            }

            // Run an EKFSLAMUpdate step
            if (noisyMeasurements.size > 0) {
                val estimatePlus = updateRelPosEKFSLAM(x_T, sigma_T, noisyMeasurements, noisyMeasurementSigmas)
                x_T = estimatePlus.mean
                sigma_T = estimatePlus.covariance
            }
        }

        truePath.add(xTrueNew)
        estimatedPath.add(x_T)

        /* ---- ---- ---- ---- Draw ---- ---- ---- ---- */
        background(0)
        noFill()
        stroke(1f, 1f, 0f)
        // Draw true landmarks
        for (landmark in landmarks) {
            circle(landmark[0], landmark[1], 10f)
        }

        // Draw the true trajectory
        stroke(0f, 1f, 0f)
        jointPoints(truePath)
        val trueState = truePath[truePath.size - 1]
        circle(trueState[0], trueState[1], 2 * SENSOR_DISTANCE)
        // Draw the laser hits
        for (laser in lasers) {
            line(trueState[0], trueState[1], laser[0], laser[1])
        }

        // Draw the estimated trajectory
        stroke(0f, 0f, 1f)
        jointPoints(estimatedPath)
        circle(estimatedPath[estimatedPath.size - 1][0], estimatedPath[estimatedPath.size - 1][1], 10f)

        // Draw the uncertainty of the robot
        visualizeCovariance(x_T[0, 0, 2, 1], sigma_T[0, 0, 2, 2])

        // Draw the uncertainty of all the landmarks in the state
        for (j in 3 until x_T.numRows step 2) {
            visualizeCovariance(x_T[j, 0, 2, 1], sigma_T[j, j, 2, 2])
        }

        surface.setTitle("Processing - FPS: ${frameRate.roundToLong()} v : ${u.a1} w : ${u.a2} #landmarks : ${(x_T.numRows - 3) / 2}")
    }

    private fun visualizeCovariance(x_T: FMatrixRMaj, sigma_T: FMatrixRMaj) {
        val sigma_T_copy = FMatrixRMaj(sigma_T)
        val decomposer = SwitchingEigenDecomposition_FDRM(2, true, 1e-6f)
        val success = decomposer.decompose(sigma_T_copy)
        if (!success) {
            throw IllegalStateException("Can't find eigen vectors/values of matrix")
        }
        val eigenValue1 = decomposer.getEigenvalue(0)
        val eigenValue2 = decomposer.getEigenvalue(1)
        val eigenVectors = EigenOps_FDRM.createMatrixV(decomposer)
        val ellipseTheta = atan2(eigenVectors[1, 0], eigenVectors[0, 0])
        val sinEllipseTheta = sin(ellipseTheta)
        val cosEllipseTheta = cos(ellipseTheta)
        val rot = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(cosEllipseTheta, -sinEllipseTheta),
                        floatArrayOf(sinEllipseTheta, cosEllipseTheta)
                )
        )
        val ellipseResolution = 20
        val ellipse = mutableListOf<FMatrixRMaj>()
        for (i in 0 until ellipseResolution) {
            // Only ellipseResolution - 1 points as the first and last points are the same (completing the loop)
            val theta = 2 * PI * i / (ellipseResolution - 1)

            // Scale by major / minor axis, then rotate and offset
            // 5.991 (=chi2inv(.95,2)) is the 95% confidence scaling bound for a 2D covariance ellipse
            // 2f * gives 99% conficence interval
            val pointOnEllipse = FMatrixRMaj(
                    arrayOf(
                            floatArrayOf(2f * sqrt((5.991 * eigenValue1.real).toFloat()) * cos(theta)),
                            floatArrayOf(2f * sqrt((5.991 * eigenValue2.real).toFloat()) * sin(theta))
                    )
            )
            ellipse.add(x_T + rot * pointOnEllipse)
        }
        stroke(1f, 0f, 0f)
        jointPoints(ellipse)
    }

    private fun jointPoints(points: MutableList<FMatrixRMaj>) {
        for (i in 1 until points.size) {
            val prevState = points[i - 1]
            val currState = points[i]
            line(prevState[0], prevState[1], currState[0], currState[1])
        }
    }

    override fun keyPressed() {
        when (keyCode) {
            PConstants.UP -> {
                u.a1 += 1f
                if (u.a1 > 20) {
                    u.a1 = 20f
                }
            }
            PConstants.DOWN -> {
                u.a1 -= 1f
                if (u.a1 < -20) {
                    u.a1 = -20f
                }
            }
            PConstants.LEFT -> {
                u.a2 = -0.15f
            }
            PConstants.RIGHT -> {
                u.a2 = 0.15f
            }
        }
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("scratch.HW2")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
