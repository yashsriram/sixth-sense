import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import robot.calibaration.RK4Integrator
import robot.sensing.ObstacleLandmarkExtractor
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator

class Calibrator : PApplet() {

    private var sim: Simulator? = null
    private var extractor: ObstacleLandmarkExtractor? = null
    private var propagatedUntil = 0f

    override fun setup() {
        val sceneName = "data/apartment.scn"
        sim = Simulator(this, sceneName)
        extractor = RANSACLeastSquares(this)
        propagatedUntil = 0f
        calibrateSigmaN()
        calibrateSigmaM()
    }

    private fun calibrateSigmaM() {
        println("Calibrating sensor measurement uncertainty")
        Simulator.GHOST_MODE = true;
        val noise = FMatrixRMaj(100, 2)
        val velocity = 100f
        val distanceThreshold = 2f
        var landmarkCounter = 0

        // get room_landmarks
        val roomLandmarks = sim!!.getPossibleLandmarks() // FIXME: replace with sim func which returns true landmark positions

        // collect samples
        println("Collecting noise samples")
        for (x in 0..99) {
            // Rotate and move the bot
            sim!!.applyControl(FMatrix2(velocity, 1f))
            Thread.sleep(500);
            val poseCopy = sim!!.getTruePose()
            val position = FMatrix2(poseCopy.a1, poseCopy.a2)
            val orientation = poseCopy.a3
            val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
            centerToHead *= sim!!.getRobotRadius()
            val tail = position - centerToHead

            val (distances, _) = sim!!.getLaserMeasurement()
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

            val (obstacles, landmarks) = extractor!!.getObservedObstaclesAndLandmarks(laserEnds, distances)

            for (landmark in landmarks) {
                for (trueLandmark in roomLandmarks) {
                    val temp = landmark - trueLandmark
                    if (temp.norm() < distanceThreshold) {
                        noise.add(landmarkCounter, 0, trueLandmark.a1 - landmark.a1)
                        noise.add(landmarkCounter, 1, trueLandmark.a2 - landmark.a2)
                        landmarkCounter+=1
                    }
                }
            }
        }

        println("Finding Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols - 1
        val nRows = landmarkCounter - 1
        for (x in 0..nCols) {
            mean.add(noise.columnWiseMean(x));
        }

        println("Finding Covariance")
        val covariance = FMatrixRMaj(2, 2)
        for (i in 0..nCols) {
            for (j in 0..nCols) {
                var variance = 0f
                for (row in 0..nRows) {
                    variance += (noise.get(row, i) - mean[i]) * (noise.get(row, j) - mean[j])
                }
                variance /= (nRows+1);
                covariance.set(i, j, variance);
            }
        }

        println(covariance)
    }

    private fun calibrateSigmaN() {
        println("Calibrating bot position uncertainty")
        Simulator.GHOST_MODE = true;
        val noise = FMatrixRMaj(100, 3)
        val numIter = 500f
        val velocity = 100f


        println("Collecting noise samples")
        for (x in 0..99) {
            val baselinePose = sim!!.getTruePose()
            // Rotate and move the bot for some time
            sim!!.applyControl(FMatrix2(velocity, 1f))
            Thread.sleep(500)
            val latestTimeElapsed = sim!!.getTimeElapsed()
            val dt = latestTimeElapsed - propagatedUntil
            propagatedUntil = latestTimeElapsed
            // Compare true pose and estimated pose
            val pose = sim!!.getTruePose()
            val estimatedPose = RK4Integrator.updatePose(baselinePose, FMatrix2(velocity, 1f), dt/numIter, (numIter).toInt())
            noise.add(x, 0, estimatedPose.a1 - pose.a1)
            noise.add(x, 1, estimatedPose.a2 - pose.a2)
            noise.add(x, 2, estimatedPose.a3 - pose.a3)
        }

        println("Finding Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols - 1
        val nRows = noise.numRows - 1
        for (x in 0..nCols) {
            mean.add(noise.columnWiseMean(x));
        }

        println("Finding Covariance")
        val covariance = FMatrixRMaj(3, 3)
        for (i in 0..nCols) {
            for (j in 0..nCols) {
                var variance = 0f
                for (row in 0..nRows) {
                    variance += (noise.get(row, i) - mean[i]) * (noise.get(row, j) - mean[j])
                }
                variance /= noise.numRows;
                covariance.set(i, j, variance);
            }
        }

        println(covariance)
    }

    override fun draw() {}
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("Calibrator")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}