package scratch

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
import kotlin.Double.Companion.POSITIVE_INFINITY
import kotlin.math.roundToLong

data class Estimate(val mean: DMatrixRMaj, val covariance: DMatrixRMaj)

class HW2 : PApplet() {
    companion object {
        const val WIDTH = 1000
        const val HEIGHT = 1000
        const val NUM_LANDMARKS = 40
        const val SENSOR_DISTANCE = 200f
    }

    private val landmarks = mutableListOf<DMatrixRMaj>()
    private val truePath = mutableListOf<DMatrixRMaj>()
    private val estimatedPath = mutableListOf<DMatrixRMaj>()

    // Source of randomness
    private val random = Random()

    // Init agent
    private val xTrue = DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(WIDTH / 2.0 - 200),
                    doubleArrayOf(HEIGHT / 2.0),
                    doubleArrayOf(-PI / 2.0)
            )
    )

    // FIXME: Initial estimate itself has some noise?
    private val std_X = 0.0
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
    private val std_M = 7.0
    private val sigma_M = CommonOps_DDRM.identity(2) * (std_M * std_M)

    // Control
    private var u = DMatrix2(20.0, 0.15)

    // Time step
    private val dt = 0.05
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

    private fun updateRelPosEKFSLAM(x_hat_t: DMatrixRMaj,
                            Sigma_x_t: DMatrixRMaj,
                            zs: MutableList<DMatrixRMaj>,
                            Sigma_ms: MutableList<DMatrixRMaj>): Estimate {
        // For each measurement, check if it matches any already in the state, and run an update for it.
        // For every unmatched measurement make sure it's sufficiently novel, then add to the state.
        var x_TPDT = DMatrixRMaj(x_hat_t);
        var sigma_TPDT = DMatrixRMaj(Sigma_x_t);
        for (i in 0 until zs.size) {
            // For each measurement
            val z = zs[i]
            val Sigma_m = Sigma_ms[i]

            // Best match quantities
            var min_distance = POSITIVE_INFINITY
            var best_H = DMatrixRMaj()
            var best_h_x_hat_0 = DMatrixRMaj()
            var best_S = DMatrixRMaj()

            // For each landmark check the Mahalanobis distance
            for (j in 3 until x_TPDT.numRows step 2) {
                val x_R_T = x_TPDT[0, 0]
                val y_R_T = x_TPDT[1, 0]
                val theta_T = x_TPDT[2, 0]
                val G_p_R = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(x_R_T),
                                doubleArrayOf(y_R_T)
                        )
                )
                val sinTheta_T = sin(theta_T.toFloat()).toDouble()
                val cosTheta_T = cos(theta_T.toFloat()).toDouble()
                val H_L_new = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(cosTheta_T, sinTheta_T),
                                doubleArrayOf(-sinTheta_T, cosTheta_T)
                        )
                )
                val x_L_T = x_TPDT[j, 0]
                val y_L_t = x_TPDT[j + 1, 0]
                val G_p_L = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(x_L_T),
                                doubleArrayOf(y_L_t)
                        )
                )
                val H_R = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(-cosTheta_T, -sinTheta_T, -sinTheta_T * (x_L_T - x_R_T) + cosTheta_T * (y_L_t - y_R_T)),
                                doubleArrayOf(sinTheta_T, -cosTheta_T, -cosTheta_T * (x_L_T - x_R_T) - sinTheta_T * (y_L_t - y_R_T))
                        )
                )

                val H = DMatrixRMaj(2, x_TPDT.numRows)
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
                val I = CommonOps_DDRM.identity(x_TPDT.numRows)

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
                val G_p_R = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(x_R_T),
                                doubleArrayOf(y_R_T)
                        )
                )
                val sinTheta_T = sin(theta_T.toFloat()).toDouble()
                val cosTheta_T = cos(theta_T.toFloat()).toDouble()
                val H_L_new = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(cosTheta_T, sinTheta_T),
                                doubleArrayOf(-sinTheta_T, cosTheta_T)
                        )
                )

                // Expected value
                // Copy previous state
                val prevX = x_TPDT
                x_TPDT = DMatrixRMaj(x_TPDT.numRows + 2, 1)
                for (t in 0 until prevX.numRows) {
                    x_TPDT[t, 0] = prevX[t, 0]
                }
                // Add new landmark estimate
                val h_x_hat_0 = H_L_new * (DMatrixRMaj(2, 1) - G_p_R)
                x_TPDT[prevX.numRows, 0, 2, 1] = H_L_new.inverse() * (z - h_x_hat_0)

                // Covariance
                // Copy previous state
                val prevSigma = sigma_TPDT
                sigma_TPDT = DMatrixRMaj(sigma_TPDT.numRows + 2, sigma_TPDT.numCols + 2)
                for (t in 0 until prevSigma.numRows) {
                    for (s in 0 until prevSigma.numCols) {
                        sigma_TPDT[t, s] = prevSigma[t, s]
                    }
                }
                // Augmentation
                val H_R = DMatrixRMaj(
                        arrayOf(
                                doubleArrayOf(-cosTheta_T, -sinTheta_T, -(0 - x_R_T) * sinTheta_T + (0 - y_R_T) * cosTheta_T),
                                doubleArrayOf(sinTheta_T, -cosTheta_T, -(0 - x_R_T) * cosTheta_T - (0 - y_R_T) * sinTheta_T)
                        )
                )
                val H_L_new_inv = H_L_new.inverse()
                val top3x3 = prevSigma[0, 0, 3, 3]
                // Top right block
                sigma_TPDT[0, prevSigma.numCols, 3, 2] =
                        top3x3 * H_R.transpose() * H_L_new_inv.transpose() * -1.0
                // Bottom left block
                sigma_TPDT[prevSigma.numRows, 0, 2, 3] =
                        H_L_new_inv * H_R * top3x3 * -1.0
                // Bottom right block
                sigma_TPDT[prevSigma.numRows, prevSigma.numCols, 2, 2] =
                        H_L_new_inv * (H_R * top3x3 * H_R.transpose() + Sigma_m) * H_L_new_inv.transpose()
                // Bottom row
                for (col in 3 until prevSigma.numCols step 2) {
                    sigma_TPDT[prevSigma.numRows, col, 2, 2] =
                            H_L_new_inv * H_R * prevSigma[0, col, 3, 2] * -1.0
                }
                // Right row
                for (row in 3 until prevSigma.numRows step 2) {
                    sigma_TPDT[row, prevSigma.numCols, 2, 2] =
                            prevSigma[row, 0, 2, 3] * H_R.transpose() * H_L_new_inv.transpose() * -1.0
                }
                continue
            }
        }

        return Estimate(mean = x_TPDT, covariance = sigma_TPDT)
    }

    override fun draw() {
        /* ---- ---- ---- ---- Update ---- ---- ---- ---- */
        iter++
        val lasers = mutableListOf<DMatrixRMaj>()
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
                if (truePositionToLandmark.norm() < SENSOR_DISTANCE) {
                    lasers.add(DMatrixRMaj(lm))
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
            circle(landmark[0].toFloat(), landmark[1].toFloat(), 10f)
        }

        // Draw the true trajectory
        stroke(0f, 1f, 0f)
        jointPoints(truePath)
        val trueState = truePath[truePath.size - 1]
        circle(trueState[0].toFloat(), trueState[1].toFloat(), 2 * SENSOR_DISTANCE)
        // Draw the laser hits
        for (laser in lasers) {
            line(trueState[0].toFloat(), trueState[1].toFloat(), laser[0].toFloat(), laser[1].toFloat())
        }

        // Draw the estimated trajectory
        stroke(0f, 0f, 1f)
        jointPoints(estimatedPath)
        circle(estimatedPath[estimatedPath.size - 1][0].toFloat(), estimatedPath[estimatedPath.size - 1][1].toFloat(), 10f)

        // Draw the uncertainty of the robot
        visualizeCovariance(x_T[0, 0, 2, 1], sigma_T[0, 0, 2, 2])

        // Draw the uncertainty of all the landmarks in the state
        for (j in 3 until x_T.numRows step 2) {
            visualizeCovariance(x_T[j, 0, 2, 1], sigma_T[j, j, 2, 2])
        }

        // Draw the uncertainty of all the landmarks in the state
        for (j in 3 until x_T.numRows step 2) {
            visualizeCovariance(x_T[j, 0, 2, 1], sigma_T[j, j, 2, 2])
        }

        surface.setTitle("Processing - FPS: ${frameRate.roundToLong()} v : ${u.a1} w : ${u.a2} #landmarks : ${(x_T.numRows - 3) / 2}")
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
            // 5.991 (=chi2inv(.95,2)) is the 95% confidence scaling bound for a 2D covariance ellipse
            // 2.0 * gives 99% conficence interval
            val pointOnEllipse = DMatrixRMaj(
                    arrayOf(
                            doubleArrayOf(2.0 * sqrt((5.991 * eigenValue1.real).toFloat()) * cos(theta)),
                            doubleArrayOf(2.0 * sqrt((5.991 * eigenValue2.real).toFloat()) * sin(theta))
                    )
            )
            ellipse.add(x_T + rot * pointOnEllipse)
        }
        stroke(1f, 0f, 0f)
        jointPoints(ellipse)
    }

    private fun jointPoints(points: MutableList<DMatrixRMaj>) {
        for (i in 1 until points.size) {
            val prevState = points[i - 1]
            val currState = points[i]
            line(prevState[0].toFloat(), prevState[1].toFloat(), currState[0].toFloat(), currState[1].toFloat())
        }
    }

    override fun keyPressed() {
        when (keyCode) {
            PConstants.UP -> {
                u.a1 += 1.0
                if (u.a1 > 20) {
                    u.a1 = 20.0
                }
            }
            PConstants.DOWN -> {
                u.a1 -= 1.0
                if (u.a1 < -20) {
                    u.a1 = -20.0
                }
            }
            PConstants.LEFT -> {
                u.a2 = -0.15
            }
            PConstants.RIGHT -> {
                u.a2 = 0.15
            }
        }
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("standalone.HW2")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}
