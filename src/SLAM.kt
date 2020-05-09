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
        const val UPDATE_THRESHOLD = 20
        const val AUGMENT_THRESHOLD = 200
        const val PERIODICAL_CLEAN_EVERY_N_AUGMENT_UPDATES = 25
        const val PERIODICAL_CLEAN_THRESHOLD = 5
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

    // Measurement covariance
    private val std_M = 1f
    private val sigma_M = CommonOps_FDRM.identity(2) * (std_M * std_M)
    private var landmarkHitMap = mutableListOf<Int>()
    private var numAugmentUpdates = 1

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
        // Keep track of the path
        truePath.add(FMatrix2(initialTruePose.a1, initialTruePose.a2))
        estimatedPath.add(FMatrix2(x_T[0], x_T[1]))
        // Start the robot
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

    private fun augmentUpdateRelPosEKFSLAM(x_T: FMatrixRMaj,
                                           sigma_x_T: FMatrixRMaj,
                                           rel_pos_msmts: MutableList<FMatrixRMaj>,
                                           sigma_Ms: MutableList<FMatrixRMaj>): Pair<FMatrixRMaj, FMatrixRMaj> {
        var x_Plus = FMatrixRMaj(x_T)
        var sigma_Plus = FMatrixRMaj(sigma_x_T)
        // For each measurement
        for (i in 0 until rel_pos_msmts.size) {
            val rel_pos_msmt = rel_pos_msmts[i]
            val sigma_M = sigma_Ms[i]

            // Best match quantities
            var min_distance = Float.POSITIVE_INFINITY
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
                val sinTheta_T = sin(theta_T)
                val cosTheta_T = cos(theta_T)
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
                if (distance[0, 0] < min_distance) {
                    min_distance = distance[0, 0]
                    best_H = H
                    best_h_x_hat_0 = h_x_hat_0
                    best_S = S
                    best_j = j
                }
            }

            // Check if it matches any already in the state, and run an update for it.
            if (min_distance <= UPDATE_THRESHOLD) {
                // EKF Update
                val K = sigma_Plus * best_H.transpose() * best_S.inverse()
                val I = CommonOps_FDRM.identity(x_Plus.numRows)
                x_Plus.plusAssign(K * (rel_pos_msmt - best_h_x_hat_0))
                val term = I - K * best_H
                sigma_Plus = term * sigma_Plus * term.transpose() + K * sigma_M * K.transpose()
                // Update number of hits map
                landmarkHitMap[(best_j - 3) / 2]++
                continue
            }

            // For unmatched measurement make sure it's sufficiently novel, then add to the state.
            if (min_distance > AUGMENT_THRESHOLD) {
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
                continue
            }
        }
        numAugmentUpdates++
        // Periodically remove measurements with very low hit rate
        if (numAugmentUpdates % PERIODICAL_CLEAN_EVERY_N_AUGMENT_UPDATES == 0) {
            kotlin.io.println("cleaning")
            // Collect indices to be kept
            val correctLandmarkIndices = mutableListOf<Int>()
            val badLandmarkMask = mutableListOf<Boolean>()
            for ((i, hits) in landmarkHitMap.withIndex()) {
                if (hits > PERIODICAL_CLEAN_THRESHOLD) {
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
            for (i in 0 until bad_sigma_Plus_Mask.numRows) {
                for (j in 0 until bad_sigma_Plus_Mask.numCols) {
                    if (((i - 3) / 2 >= 0 && badLandmarkMask[(i - 3) / 2])
                            || ((j - 3) / 2 >= 0 && badLandmarkMask[(j - 3) / 2])) {
                        bad_sigma_Plus_Mask[i, j] = 1f
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
            val (x_Plus, sigma_Plus) = augmentUpdateRelPosEKFSLAM(x_T, sigma_T, rel_pos_msmts, sigma_Ms)
            x_T = x_Plus
            sigma_T = sigma_Plus
            // Update last measured
            lastMeasured = timestamp
        }

        // Keep track of path
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
