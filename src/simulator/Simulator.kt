package simulator

import extensions.minus
import extensions.plus
import extensions.times
import extensions.timesAssign
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix3
import processing.core.PApplet
import java.io.File
import java.io.FileNotFoundException
import java.util.*
import kotlin.collections.HashSet
import kotlin.math.max
import kotlin.math.min
import kotlin.system.exitProcess

class Simulator(private val applet: PApplet, sceneFilepath: String) {
    companion object {
        const val ROBOT_LOOP_DURATION = 100f / 6
        const val ROBOT_LOOP_DT = 1e-2f
        const val CONTROL_FREQ = 1
        const val LASER_SCAN_FREQUENCY = 10
        const val SCALE = 100f
        var DRAW_OBSTACLES = true
        var GHOST_MODE = false
    }

    // Simulation
    private var isPaused = false

    // Environment
    private val obstacles: MutableList<Obstacle> = ArrayList()

    // Robot
    private val robot: Robot
    private val initialPose: FMatrix3
    private var robotTimeElapsed = 0f

    init {
        // Read scene
        val fileContents: MutableList<List<String>> = ArrayList()
        try {
            val delimiter = " "
            val file = File(sceneFilepath)
            val scanner = Scanner(file)
            while (scanner.hasNextLine()) {
                val line = scanner.nextLine()
                val tokens = line.split(delimiter.toRegex()).toTypedArray()
                fileContents.add(ArrayList(listOf(*tokens)))
            }
            scanner.close()
        } catch (e: FileNotFoundException) {
            e.printStackTrace()
            exitProcess(-1)
        }
        assert(fileContents.size > 0)

        // Every line after 1st one in the file is a line segment
        val minCorner = FMatrix2(Float.POSITIVE_INFINITY, Float.POSITIVE_INFINITY)
        val maxCorner = FMatrix2(Float.NEGATIVE_INFINITY, Float.NEGATIVE_INFINITY)
        for (i in 1 until fileContents.size) {
            val lineFeatureTokens = fileContents[i]
            assert(lineFeatureTokens.size == 4)
            minCorner.a1 = min(minCorner.a1, lineFeatureTokens[0].toFloat())
            minCorner.a1 = min(minCorner.a1, lineFeatureTokens[2].toFloat())
            minCorner.a2 = min(minCorner.a2, lineFeatureTokens[1].toFloat())
            minCorner.a2 = min(minCorner.a2, lineFeatureTokens[3].toFloat())

            maxCorner.a1 = max(maxCorner.a1, lineFeatureTokens[0].toFloat())
            maxCorner.a1 = max(maxCorner.a1, lineFeatureTokens[2].toFloat())
            maxCorner.a2 = max(maxCorner.a2, lineFeatureTokens[1].toFloat())
            maxCorner.a2 = max(maxCorner.a2, lineFeatureTokens[3].toFloat())
        }
        val center = minCorner + (maxCorner - minCorner) * 0.5f

        // Loading obstacles with scaling and center shifting
        for (i in 1 until fileContents.size) {
            val lineTokens = fileContents[i]
            assert(lineTokens.size == 4)
            val p1 = FMatrix2(lineTokens[0].toFloat(), lineTokens[1].toFloat()) - center
            p1 *= SCALE
            val p2 = FMatrix2(lineTokens[2].toFloat(), lineTokens[3].toFloat()) - center
            p2 *= SCALE
            obstacles.add(LineSegmentObstacle(applet, p1, p2))
        }

        // Load robot pose with scaling and center shifting
        val poseTokens = fileContents[0]
        assert(poseTokens.size == 4)
        initialPose = FMatrix3(
                (poseTokens[0].toFloat() - center.a1) * SCALE,
                (poseTokens[1].toFloat() - center.a2) * SCALE,
                poseTokens[2].toFloat())
        robot = Robot(
                applet,
                poseTokens[3].toFloat() * SCALE,
                initialPose,
                true
        )

        // Start the robot loop in a new thread
        val robotLoop = Thread(Runnable { robotLoop() })
        robotLoop.start()
    }

    private fun robotLoop() {
        var iteration = 0
        while (robot.isRunning) {
            val isPausedLocal: Boolean
            synchronized(isPaused) {
                isPausedLocal = isPaused
            }
            if (isPausedLocal) {
                continue
            }
            if (iteration % CONTROL_FREQ == 0) {
                robot.updatePose(ROBOT_LOOP_DT)
                if (!GHOST_MODE) {
                    for (line in obstacles) {
                        if (robot.isCrashing(line)) {
                            println("Robot: \"Oh No! I crashed!!!!\"")
                            robot.isRunning = false
                        }
                    }
                }
            }
            if (iteration % LASER_SCAN_FREQUENCY == 0) {
                robot.updateSense(obstacles)
            }
            iteration++
            synchronized(robotTimeElapsed) {
                robotTimeElapsed += ROBOT_LOOP_DT
            }
            try {
                Thread.sleep(ROBOT_LOOP_DURATION.toLong())
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
    }

    /* User callable */
    fun getTimeElapsed(): Float {
        var timeElapsed: Float
        synchronized(robotTimeElapsed) {
            timeElapsed = robotTimeElapsed
        }
        return timeElapsed
    }

    fun getPossibleLandmarks(): Set<FMatrix2> {
        val lineEnds = HashSet<FMatrix2>()
        for (obstacle in obstacles) {
            lineEnds += obstacle.possibleLandmarks()
        }
        return lineEnds
    }

    fun getTruePose(): FMatrix3 {
        return robot.getTruePose()
    }

    fun getRobotRadius(): Float {
        return robot.radius
    }

    fun getLaserMeasurement(): Pair<List<Float>, Long> {
        return robot.laserSensor.getDistances()
    }

    fun getCurrentControl(): FMatrix2 {
        return robot.getCurrentControl()
    }

    fun applyControl(control: FMatrix2) {
        robot.applyControl(control)
    }

    fun togglePaused() {
        synchronized(isPaused) {
            isPaused = !isPaused
        }
    }

    fun draw() {
        if (DRAW_OBSTACLES) {
            // Draw obstacles
            for (l in obstacles) {
                l.draw()
            }
        }
        // Draw robot
        robot.draw()
    }
}