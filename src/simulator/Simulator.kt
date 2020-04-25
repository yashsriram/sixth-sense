package simulator

import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix3
import processing.core.PApplet
import java.io.File
import java.io.FileNotFoundException
import java.util.*
import kotlin.math.max
import kotlin.math.min
import kotlin.system.exitProcess

class Simulator(private val applet: PApplet, sceneFilepath: String) {
    companion object {
        // Simulator settings
        var CONTROL_FREQ = 1
        var LASER_SCAN_FREQUENCY = 10

        // Graphics
        var SCALE = 100.0
        var DRAW_OBSTACLES = true
    }

    // Environment
    private val lines: MutableList<Obstacle> = ArrayList()

    // Robot
    private val robot: Robot
    val initialPose: DMatrix3

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
        val minCorner = DMatrix2(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY)
        val maxCorner = DMatrix2(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY)
        for (i in 1 until fileContents.size) {
            val lineFeatureTokens = fileContents[i]
            assert(lineFeatureTokens.size == 4)
            minCorner.a1 = min(minCorner.a1, lineFeatureTokens[0].toDouble())
            minCorner.a1 = min(minCorner.a1, lineFeatureTokens[2].toDouble())
            minCorner.a2 = min(minCorner.a2, lineFeatureTokens[1].toDouble())
            minCorner.a2 = min(minCorner.a2, lineFeatureTokens[3].toDouble())

            maxCorner.a1 = max(maxCorner.a1, lineFeatureTokens[0].toDouble())
            maxCorner.a1 = max(maxCorner.a1, lineFeatureTokens[2].toDouble())
            maxCorner.a2 = max(maxCorner.a2, lineFeatureTokens[1].toDouble())
            maxCorner.a2 = max(maxCorner.a2, lineFeatureTokens[3].toDouble())
        }
        val center = minCorner + (maxCorner - minCorner) * 0.5

        // Loading lines with scaling and center shifting
        for (i in 1 until fileContents.size) {
            val lineTokens = fileContents[i]
            assert(lineTokens.size == 4)
            val p1 = DMatrix2(lineTokens[0].toDouble(), lineTokens[1].toDouble()) - center
            p1 *= SCALE
            val p2 = DMatrix2(lineTokens[2].toDouble(), lineTokens[3].toDouble()) - center
            p2 *= SCALE
            lines.add(LineSegmentObstacle(applet, p1, p2))
        }

        // Load robot pose with scaling and center shifting
        val poseTokens = fileContents[0]
        assert(poseTokens.size == 4)
        // Fork off the main simulation loop
        initialPose = DMatrix3(
                (poseTokens[0].toDouble() - center.a1) * SCALE,
                (poseTokens[1].toDouble() - center.a2) * SCALE,
                poseTokens[2].toDouble())
        robot = Robot(
                applet,
                poseTokens[3].toDouble() * SCALE,
                initialPose,
                true
        )
        val robotLoop = Thread(Runnable { robotLoop() })
        robotLoop.start()
    }

    private fun robotLoop() {
        val loopDuration: Long = 10
        val loopDt = 1e-3 * loopDuration
        var iteration = 0
        while (robot.isRunning) {
            if (iteration % CONTROL_FREQ == 0) {
                robot.updatePose(loopDt)
                for (line in lines) {
                    if (robot.isCrashing(line)) {
                        println("Robot: \"Oh No! I crashed!!!!\"")
                        robot.isRunning = false
                    }
                }
            }
            if (iteration % LASER_SCAN_FREQUENCY == 0) {
                robot.updateSense(lines)
            }
            iteration++
            try {
                Thread.sleep(loopDuration)
            } catch (e: InterruptedException) {
                e.printStackTrace()
            }
        }
    }

    /* User callable */
    val truePose: DMatrix3
        get() = robot.getTruePose()

    val robotLength: Double
        get() = robot.length

    val laserScan: List<Double>
        get() = robot.laserSensor.getMeasurements()

    val currentControl: DMatrix2
        get() = robot.getCurrentControl()

    fun applyControl(control: DMatrix2) {
        robot.applyControl(control)
    }

    fun draw() {
        if (DRAW_OBSTACLES) {
            // Draw obstacles
            for (l in lines) {
                l.draw()
            }
        }
        // Draw robot
        robot.draw()
    }
}