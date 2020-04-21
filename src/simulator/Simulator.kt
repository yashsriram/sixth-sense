package simulator

import extensions.timesAssign
import math.Vec2
import math.Vec3
import org.ejml.data.DMatrix2
import processing.core.PApplet
import java.io.File
import java.io.FileNotFoundException
import java.util.*

class Simulator(private val applet: PApplet, sceneFilepath: String?) {
    companion object {
        // Simulator settings
        var CONTROL_FREQ = 1
        var LASER_SCAN_FREQUENCY = 10

        // Graphics
        var SCALE = 100.0
    }

    // Environment
    private val lines: MutableList<Landmark> = ArrayList()

    // Robot
    private val robot: Robot

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
                fileContents.add(ArrayList(Arrays.asList(*tokens)))
            }
            scanner.close()
        } catch (e: FileNotFoundException) {
            e.printStackTrace()
            System.exit(-1)
        }
        assert(fileContents.size > 0)

        // Every line after 1st one in the file is a line segment
        for (i in 1 until fileContents.size) {
            val lineFeatureTokens = fileContents[i]
            assert(lineFeatureTokens.size == 4)
            val p1 = DMatrix2(lineFeatureTokens[0].toDouble(), lineFeatureTokens[1].toDouble())
            p1 *= SCALE
            val p2 = DMatrix2(lineFeatureTokens[2].toDouble(), lineFeatureTokens[3].toDouble())
            p2 *= SCALE
            lines.add(LineSegment(applet, p1, p2))
        }

        // Load robot pose
        val poseTokens = fileContents[0]
        assert(poseTokens.size == 4)
        // Fork off the main simulation loop
        robot = Robot(
                applet,
                poseTokens[3].toDouble() * SCALE,
                Vec3.of(
                        poseTokens[0].toDouble() * SCALE,
                        poseTokens[1].toDouble() * SCALE, poseTokens[2].toDouble()),
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

    val laserScan: List<Double>
        get() = robot.laser.getMeasurements()

    val currentControl: Vec2
        get() = robot.getCurrentControl()

    fun applyControl(control: Vec2?) {
        robot.applyControl(control!!)
    }

    fun draw() {
        // Draw landmarks
        for (l in lines) {
            l.draw()
        }
        // Draw robot
        robot.draw()
    }
}