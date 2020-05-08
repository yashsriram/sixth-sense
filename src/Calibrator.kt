import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import robot.calibaration.RK4Integrator
import robot.sensing.ObstacleLandmarkExtractor
import simulator.LaserSensor
import simulator.Simulator
import kotlin.collections.ArrayList
class Calibrator: PApplet() {

    private var sim: Simulator? = null
    private var extractor: ObstacleLandmarkExtractor? = null

    override fun setup() {
        val sceneName = "data/apartment.scn"
        sim = Simulator(this, sceneName)
        calibrate()
        calibrate_Measurement()
    }

    private fun calibrate_Measurement(){
        println("Calibrating sensor measurement covariance")
        Simulator.GHOST_MODE = true;
        val noise = FMatrixRMaj(100,2)
        val velocity = 100f
        val distanceThreshold = 20f

        // get room_landmarks
        val roomLandmarks: MutableList<FMatrix2> = ArrayList() // FIXME: replace with sim func which returns true landmark positions

        // collect samples
        println("Collecting noise samples")
        for(x in 0..99){

            // rotate and move the bot
            sim!!.applyControl(FMatrix2(velocity, 0f))
            // get room landmarks
            Thread.sleep(500);
            val poseCopy = sim!!.truePose
            val position = FMatrix2(poseCopy.a1, poseCopy.a2)
            val orientation = poseCopy.a3
            val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
            centerToHead *= sim!!.robotRadius
            val tail = position - centerToHead

            val distances = sim!!.laserDistances
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

            for (landmark in landmarks){
                for (trueLandmark in roomLandmarks){
                    val temp = landmark - trueLandmark
                    if(temp.norm() < distanceThreshold){
                        noise.add(x,1, trueLandmark.a1 - landmark.a1)
                        noise.add(x,0, trueLandmark.a1 - landmark.a1)
                    }
                }
            }
        }

        // get column means
        println("Finding Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols-1
        val nRows = noise.numRows-1
        for(x in 0..nCols){
            mean.add(noise.columnWiseMean(x));
        }

        // calculate covariance
        println("Finding Covariance")
        val covariance: FMatrixRMaj = FMatrixRMaj(2,2)
        for(i in 0..nCols){
            for(j in 0..nCols){
                var variance = 0f
                for(row in 0..nRows){
                    variance += (noise.get(row, i) - mean[i])*(noise.get(row, j) - mean[j])
                }
                variance /= noise.numRows;
                covariance.set(i, j, variance);
            }
        }

        // print covariance
        for(i in 0..1){
            for(j in 0..1){
                PApplet.print(covariance.get(i, j), ",")
            }
            PApplet.println()
        }
    }

    private fun calibrate(){
        println("Calibrating bot position covariance")
        Simulator.GHOST_MODE = true;
        val noise = FMatrixRMaj(100,3)
        val dt = 0.01f;
        val velocity = 100f
        // collect samples

        println("Collecting noise samples")
        for(x in 0..99){

            // move the bot forward
            val baselinePose = sim!!.truePose

            // rotate and move the bot
            sim!!.applyControl(FMatrix2(velocity, 0f))
            Thread.sleep(500);
            val pose = sim!!.truePose
            val estimatedPose = RK4Integrator.updatePose(baselinePose, FMatrix2(velocity, 0f), dt, PApplet.parseInt(dt * 500f));
            noise.add(x,0, estimatedPose.a1 - pose.a1)
            noise.add(x,1, estimatedPose.a2 - pose.a2)
            noise.add(x,2, estimatedPose.a3 - pose.a3)
        }

        // get column means
        println("Finding Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols-1
        val nRows = noise.numRows-1
        for(x in 0..nCols){
            mean.add(noise.columnWiseMean(x));
        }

        // calculate covariance
        println("Finding Covariance")
        val covariance: FMatrixRMaj = FMatrixRMaj(3,3)
        for(i in 0..nCols){
            for(j in 0..nCols){
                var variance = 0f
                for(row in 0..nRows){
                    variance += (noise.get(row, i) - mean[i])*(noise.get(row, j) - mean[j])
                }
                variance /= noise.numRows;
                covariance.set(i, j, variance);
            }
        }

        // print covariance
        for(i in 0..2){
            for(j in 0..2){
                PApplet.print(covariance.get(i, j), ",")
            }
            PApplet.println()
        }
    }

    override fun draw() {
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("Calibrator")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}