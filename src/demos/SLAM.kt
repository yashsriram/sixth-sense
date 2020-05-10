package demos

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
import processing.core.PApplet
import robot.RK4Integrator

class SLAM {
    private var landmarkHitMap = mutableListOf<Int>()
    private var numAugmentUpdates = 1

    fun reset() {
        landmarkHitMap = mutableListOf()
        numAugmentUpdates = 1
    }

    fun propagateEKFSLAM(x_T: FMatrixRMaj,
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
                        floatArrayOf(1f, 0f, -dt * v * PApplet.sin(theta_t)),
                        floatArrayOf(0f, 1f, dt * v * PApplet.cos(theta_t)),
                        floatArrayOf(0f, 0f, 1f)
                )
        )
        val N = FMatrixRMaj(
                arrayOf(
                        floatArrayOf(dt * PApplet.cos(theta_t), 0f),
                        floatArrayOf(dt * PApplet.sin(theta_t), 0f),
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

    fun augmentUpdateRelPosEKFSLAM(x_T: FMatrixRMaj,
                                   sigma_x_T: FMatrixRMaj,
                                   rel_pos_msmts: MutableList<FMatrixRMaj>,
                                   sigma_Ms: MutableList<FMatrixRMaj>): Pair<FMatrixRMaj, FMatrixRMaj> {
        var x_Plus = FMatrixRMaj(x_T)
        var sigma_Plus = FMatrixRMaj(sigma_x_T)
        val newLandmarks = mutableListOf<Int>()
        // For each measurement
        for (i in 0 until rel_pos_msmts.size) {
            val rel_pos_msmt = rel_pos_msmts[i]
            val sigma_M = sigma_Ms[i]

            // Best match quantities
            var minDistance = Float.POSITIVE_INFINITY
            var best_H = FMatrixRMaj()
            var best_h_x_hat_0 = FMatrixRMaj()
            var best_S = FMatrixRMaj()
            var best_j = -1

            // For each landmark check the Mahalanobis distance
            for (j in 3 until x_Plus.numRows step 2) {
                val x_R_T = x_Plus[0, 0]
                val y_R_T = x_Plus[1, 0]
                val theta_T = x_Plus[2, 0]
                val G_p_R = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(x_R_T),
                                floatArrayOf(y_R_T)
                        )
                )
                val sinTheta_T = PApplet.sin(theta_T)
                val cosTheta_T = PApplet.cos(theta_T)
                val H_L_new = FMatrixRMaj(
                        arrayOf(
                                floatArrayOf(cosTheta_T, sinTheta_T),
                                floatArrayOf(-sinTheta_T, cosTheta_T)
                        )
                )
                val x_L_T = x_Plus[j, 0]
                val y_L_t = x_Plus[j + 1, 0]
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

                val H = FMatrixRMaj(2, x_Plus.numRows)
                H[0, 0, 2, 3] = H_R
                H[0, j, 2, 2] = H_L_new

                val S = H * sigma_Plus * H.transpose() + sigma_M
                val h_x_hat_0 = H_L_new * (G_p_L - G_p_R)
                val residue = rel_pos_msmt - h_x_hat_0
                val distance = residue.transpose() * S.inverse() * residue

                assert(distance.numElements == 1)

                // Track the most likely landmark
                if (distance[0, 0] < minDistance) {
                    minDistance = distance[0, 0]
                    best_H = H
                    best_h_x_hat_0 = h_x_hat_0
                    best_S = S
                    best_j = j
                }
            }

            // Check if it matches any already in the state, and run an update for it.
            if (minDistance <= Simulation.UPDATE_THRESHOLD) {
                // EKF Update
                val K = sigma_Plus * best_H.transpose() * best_S.inverse()
                val I = CommonOps_FDRM.identity(x_Plus.numRows)
                x_Plus.plusAssign(K * (rel_pos_msmt - best_h_x_hat_0))
                val term = I - K * best_H
                sigma_Plus = term * sigma_Plus * term.transpose() + K * sigma_M * K.transpose()
                // Update number of hits map
                landmarkHitMap[(best_j - 3) / 2]++
            } else if (minDistance > Simulation.AUGMENT_THRESHOLD) {
                newLandmarks.add(i)
            }
        }
        // For unmatched measurement make sure it's sufficiently novel, then add to the state.
        for (i in newLandmarks) {
            val rel_pos_msmt = rel_pos_msmts[i]
            val sigma_M = sigma_Ms[i]

            // EKF Augment
            val x_R_T = x_Plus[0, 0]
            val y_R_T = x_Plus[1, 0]
            val theta_T = x_Plus[2, 0]
            val G_p_R = FMatrixRMaj(
                    arrayOf(
                            floatArrayOf(x_R_T),
                            floatArrayOf(y_R_T)
                    )
            )
            val sinTheta_T = PApplet.sin(theta_T)
            val cosTheta_T = PApplet.cos(theta_T)
            val H_L_new = FMatrixRMaj(
                    arrayOf(
                            floatArrayOf(cosTheta_T, sinTheta_T),
                            floatArrayOf(-sinTheta_T, cosTheta_T)
                    )
            )

            // Expected value
            // Copy previous state
            val prevX = x_Plus
            x_Plus = FMatrixRMaj(x_Plus.numRows + 2, 1)
            for (t in 0 until prevX.numRows) {
                x_Plus[t, 0] = prevX[t, 0]
            }
            // Add new landmark estimate
            val h_x_hat_0 = H_L_new * (FMatrixRMaj(2, 1) - G_p_R)
            x_Plus[prevX.numRows, 0, 2, 1] = H_L_new.inverse() * (rel_pos_msmt - h_x_hat_0)

            // Covariance
            // Copy previous state
            val prevSigma = sigma_Plus
            sigma_Plus = FMatrixRMaj(sigma_Plus.numRows + 2, sigma_Plus.numCols + 2)
            for (t in 0 until prevSigma.numRows) {
                for (s in 0 until prevSigma.numCols) {
                    sigma_Plus[t, s] = prevSigma[t, s]
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
            sigma_Plus[0, prevSigma.numCols, 3, 2] =
                    top3x3 * H_R.transpose() * H_L_new_inv.transpose() * -1f
            // Bottom left block
            sigma_Plus[prevSigma.numRows, 0, 2, 3] =
                    H_L_new_inv * H_R * top3x3 * -1f
            // Bottom right block
            sigma_Plus[prevSigma.numRows, prevSigma.numCols, 2, 2] =
                    H_L_new_inv * (H_R * top3x3 * H_R.transpose() + sigma_M) * H_L_new_inv.transpose()
            // Bottom row
            for (col in 3 until prevSigma.numCols step 2) {
                sigma_Plus[prevSigma.numRows, col, 2, 2] =
                        H_L_new_inv * H_R * prevSigma[0, col, 3, 2] * -1f
            }
            // Right row
            for (row in 3 until prevSigma.numRows step 2) {
                sigma_Plus[row, prevSigma.numCols, 2, 2] =
                        prevSigma[row, 0, 2, 3] * H_R.transpose() * H_L_new_inv.transpose() * -1f
            }
            // Add to number of hits map
            landmarkHitMap.add(1)
        }
        numAugmentUpdates++
        // Periodically remove measurements with very low hit rate
        if (numAugmentUpdates % Simulation.PERIODICAL_CLEAN_EVERY_N_AUGMENT_UPDATES == 0) {
            println("cleaning")
            // Collect indices to be kept
            val correctLandmarkIndices = mutableListOf<Int>()
            val badLandmarkMask = mutableListOf<Boolean>()
            for ((i, hits) in landmarkHitMap.withIndex()) {
                if (hits > Simulation.PERIODICAL_CLEAN_THRESHOLD) {
                    correctLandmarkIndices.add(i)
                    badLandmarkMask.add(false)
                } else {
                    badLandmarkMask.add(true)
                }
            }
            // Make a corrected copy
            val landmarkHitMap_Corrected = mutableListOf<Int>()
            // x_Plus_Corrected
            val x_Plus_Corrected = FMatrixRMaj(3 + 2 * correctLandmarkIndices.size, 1)
            x_Plus_Corrected[0, 0, 3, 1] = x_Plus[0, 0, 3, 1]
            for ((i, correctLandmarkIndex) in correctLandmarkIndices.withIndex()) {
                // Add correct landmark hits
                landmarkHitMap_Corrected.add(landmarkHitMap[correctLandmarkIndex])
                // Add correct landmarks to corrected state
                x_Plus_Corrected[3 + 2 * i, 0, 2, 1] = x_Plus[3 + 2 * correctLandmarkIndex, 0, 2, 1]
            }
            // sigma_Plus_Corrected
            val bad_sigma_Plus_Mask = sigma_Plus.createLike()
            if (badLandmarkMask.isNotEmpty()) {
                for (i in 0 until bad_sigma_Plus_Mask.numRows) {
                    for (j in 0 until bad_sigma_Plus_Mask.numCols) {
                        if (((i - 3) / 2 >= 0 && badLandmarkMask[(i - 3) / 2])
                                || ((j - 3) / 2 >= 0 && badLandmarkMask[(j - 3) / 2])) {
                            bad_sigma_Plus_Mask[i, j] = 1f
                        }
                    }
                }
            }
            val sigma_Plus_Corrected = FMatrixRMaj(3 + 2 * correctLandmarkIndices.size, 3 + 2 * correctLandmarkIndices.size)
            var corrected_i = 0
            for (i in 0 until bad_sigma_Plus_Mask.numRows) {
                var corrected_j = 0
                for (j in 0 until bad_sigma_Plus_Mask.numCols) {
                    if (bad_sigma_Plus_Mask[i, j] == 0f) {
                        sigma_Plus_Corrected[corrected_i, corrected_j] = sigma_Plus[i, j]
                        corrected_j++
                        if (corrected_j == 3 + 2 * correctLandmarkIndices.size) {
                            corrected_i++
                        }
                    }
                }
            }
            // Update variables
            landmarkHitMap = landmarkHitMap_Corrected
            x_Plus = x_Plus_Corrected
            sigma_Plus = sigma_Plus_Corrected
        }

        return Pair(x_Plus, sigma_Plus)
    }
}