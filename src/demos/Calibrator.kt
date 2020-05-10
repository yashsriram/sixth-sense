package demos

import camera.QueasyCam
import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix3
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import processing.core.PConstants
import robot.RK4Integrator
import robot.planning.HitGrid
import robot.sensing.ObstacleLandmarkExtractor
import robot.sensing.RANSACLeastSquares
import simulator.LaserSensor
import simulator.Simulator
import kotlin.math.roundToInt

class Calibrator : PApplet() {

    companion object {
        const val WIDTH = 900
        const val HEIGHT = 900
        var DRAW_OBSTACLES_LANDMARKS = true
    }

    private var sim: Simulator? = null
    private var extractor: ObstacleLandmarkExtractor? = null
    private var cam: QueasyCam? = null
    private var hitGrid: HitGrid? = null
    private var propagatedUntil = 0f
    private var velocity = 100
    private var distanceThreshold = 2f
    private var baselinePose = FMatrix3()
    private var numIter = 500f
    private var measurementNoise = FMatrixRMaj(100, 2)
    private var measurementNoiseCounter = 0
    private var positionNoise = FMatrixRMaj(100, 3)
    private var positionNoiseCounter = 0
    private var randomAngularVelocity = 0f

    override fun settings() {
        size(WIDTH, HEIGHT, PConstants.P3D)
    }

    override fun setup() {
        surface.setTitle("Processing")
        val sceneName = "data/apartment.scn"
        sim = Simulator(this, sceneName)
        extractor = RANSACLeastSquares(this)
        propagatedUntil = 0f
        colorMode(PConstants.RGB, 1.0f)
        rectMode(PConstants.CENTER)
        cam = QueasyCam(this)
        reset()
    }

    private fun reset() {
        val sceneName = "data/simple_rectangle.scn"
        sim = Simulator(this, sceneName)
        hitGrid = HitGrid(this, FMatrix2(-1000f, -1000f), FMatrix2(1000f, 1000f), 1000, 1000)
        extractor = RANSACLeastSquares(this)
        Simulator.GHOST_MODE = true
        baselinePose = sim!!.getTruePose();
        velocity = (50..150).random()
        randomAngularVelocity = (50..100).random() / 100f
    }

    private fun calculateCovariance(noise: FMatrixRMaj) {
        println("Finding Noise Mean")
        val mean: MutableList<Float> = ArrayList()
        val nCols = noise.numCols - 1
        val nRows = noise.numRows - 1
        for (x in 0..nCols) {
            mean.add(noise.columnWiseMean(x));
        }

        println("Finding Covariance")
        val covariance = FMatrixRMaj(nCols + 1, nCols + 1)
        for (i in 0..nCols) {
            for (j in 0..nCols) {
                var variance = 0f
                for (row in 0..nRows) {
                    variance += (noise.get(row, i) - mean[i]) * (noise.get(row, j) - mean[j])
                }
                variance /= (nRows + 1);
                covariance.set(i, j, variance);
            }
        }

        println(covariance)
    }

    override fun draw() {
        /* Clear screen */
        sim!!.applyControl(FMatrix2(parseFloat(velocity), randomAngularVelocity))
        val latestTimeElapsed = sim!!.getTimeElapsed()
        val dt = latestTimeElapsed - propagatedUntil
        background(0)

        /* Update */
        val poseCopy = sim!!.getTruePose() // FIXME: should use estimated pose here later
        val position = FMatrix2(poseCopy.a1, poseCopy.a2)
        val orientation = poseCopy.a3
        val centerToHead = FMatrix2(kotlin.math.cos(orientation), kotlin.math.sin(orientation))
        centerToHead *= sim!!.getRobotRadius()
        val tail = position - centerToHead

        val (distances, timestamp) = sim!!.getLaserMeasurement()
        val laserEnds: MutableList<FMatrix2> = java.util.ArrayList(LaserSensor.COUNT)
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

        if (dt > 0.2) {
            propagatedUntil = latestTimeElapsed
            val pose = sim!!.getTruePose()
            val estimatedPose = RK4Integrator.updatePose(baselinePose, FMatrix2(parseFloat(velocity), 1f), dt / numIter, (numIter).toInt())
            positionNoise.add(positionNoiseCounter, 0, estimatedPose.a1 - pose.a1)
            positionNoise.add(positionNoiseCounter, 1, estimatedPose.a2 - pose.a2)
            positionNoise.add(positionNoiseCounter, 2, estimatedPose.a3 - pose.a3)
            positionNoiseCounter+=1
            baselinePose = pose

            if (measurementNoiseCounter < 100) {
                val roomLandmarks = sim!!.getPossibleLandmarks()
                for (landmark in landmarks) {
                    for (trueLandmark in roomLandmarks) {
                        val temp = landmark - trueLandmark
                        if (temp.norm() < distanceThreshold) {
                            measurementNoise.add(measurementNoiseCounter, 0, trueLandmark.a1 - landmark.a1)
                            measurementNoise.add(measurementNoiseCounter, 1, trueLandmark.a2 - landmark.a2)
                            measurementNoiseCounter += 1
                        }
                    }
                }
            }

            if (positionNoiseCounter == 100) {
                println("Calculating sigmaN")
                calculateCovariance(positionNoise)

                println("Calculating sigmaM")
                calculateCovariance(measurementNoise)

                println("Exiting Program")
                this.exit()
            }

            velocity = (50..150).random()
            randomAngularVelocity = (50..100).random() / 100f
        }

        /* Draw */
        sim!!.draw()
//        hitGrid!!.draw()

        surface.setTitle("Processing - FPS: ${frameRate.roundToInt()}" +
                " extractor=${extractor!!.getName()}" +
                " #obs=${obstacles.size} #land=${landmarks.size}"
        )
    }
}

fun main(passedArgs: Array<String>) {
    val appletArgs = arrayOf("demos.Calibrator")
    if (passedArgs != null) {
        PApplet.main(PApplet.concat(appletArgs, passedArgs))
    } else {
        PApplet.main(appletArgs)
    }
}