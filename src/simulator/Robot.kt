package simulator

import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix3
import processing.core.PApplet
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class Robot internal constructor(private val applet: PApplet, val length: Double, truePose: DMatrix3, var isRunning: Boolean) {
    companion object {
        const val MAX_LINEAR_ACCELERATION = 20.0
        const val MAX_ANGULAR_ACCELERATION = 0.5
        const val LINEAR_VELOCITY_ERROR_LIMIT = 2.0
        const val ANGULAR_VELOCITY_ERROR_LIMIT = 0.1
        private fun getChangeInPose(pose: DMatrix3, control: DMatrix2): DMatrix3 {
            val changeInPose = DMatrix3()
            changeInPose.a1 = control.a1 * cos(pose.a3)
            changeInPose.a2 = control.a1 * sin(pose.a3)
            changeInPose.a3 = control.a2
            return changeInPose
        }

        private fun getPositionFromPose(pose: DMatrix3): DMatrix2 {
            return DMatrix2(pose.a1, pose.a2)
        }
    }

    // Multi-thread access
    private val truePose = DMatrix3(truePose)
    private val goalControl = DMatrix2()
    private val currentControl = DMatrix2()

    // Sensors
    val laserSensor: LaserSensor = LaserSensor(applet)

    fun updatePose(dt: Double) {
        var noisyControl = DMatrix2()
        synchronized(currentControl) {
            synchronized(goalControl) {

                // Clamp acceleration
                val controlDiff = goalControl - currentControl
                if (abs(controlDiff.a1) > MAX_LINEAR_ACCELERATION * dt) {
                    controlDiff.a1 = sign(controlDiff.a1) * MAX_LINEAR_ACCELERATION * dt
                }
                if (abs(controlDiff.a2) > MAX_ANGULAR_ACCELERATION * dt) {
                    controlDiff.a2 = sign(controlDiff.a2) * MAX_ANGULAR_ACCELERATION * dt
                }
                currentControl += controlDiff
                noisyControl = DMatrix2(currentControl)
            }
        }

        // Only apply noise if we're trying to move
        if (noisyControl.squaredNorm() != 0.0) {
            noisyControl.a1 *= 1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT)
            noisyControl.a2 *= 1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT)
        }

        // Run the dynamics via RK4
        synchronized(truePose) {
            val k1 = getChangeInPose(truePose, noisyControl)
            val x2 = truePose + k1 * (0.5f * dt)
            val k2 = getChangeInPose(x2, noisyControl)
            val x3 = truePose + k2 * (0.5f * dt)
            val k3 = getChangeInPose(x3, noisyControl)
            val x4 = truePose + k3 * dt
            val k4 = getChangeInPose(x4, noisyControl)
            val avgChangeInPose = DMatrix3()
            avgChangeInPose += k1
            avgChangeInPose += k2 * 2.0
            avgChangeInPose += k3 * 2.0
            avgChangeInPose += k4
            avgChangeInPose *= dt / 6.0
            truePose += avgChangeInPose
        }
    }

    fun updateSense(obstacles: List<Obstacle>) {
        // Move the center of the scanner back from the center of the robot
        val truePosition: DMatrix2
        val orientation: Double
        synchronized(truePose) {
            truePosition = getPositionFromPose(truePose)
            orientation = truePose.a3
        }
        val centerToHead = DMatrix2(cos(orientation), sin(orientation))
        centerToHead *= 0.5 * length
        val laserSource = truePosition - centerToHead
        laserSensor.updateLaserScan(laserSource, orientation, obstacles)
    }

    fun isCrashing(obstacle: Obstacle): Boolean {
        synchronized(truePose) {
            return obstacle.shortestDistanceFrom(getPositionFromPose(truePose)) < length / 2
        }
    }

    /* User callable */
    fun getTruePose(): DMatrix3 {
        val answer = DMatrix3()
        synchronized(truePose) { answer.set(truePose) }
        return answer
    }

    fun getCurrentControl(): DMatrix2 {
        val answer = DMatrix2()
        synchronized(currentControl) { answer.set(currentControl) }
        return answer
    }

    fun applyControl(control: DMatrix2) {
        synchronized(goalControl) { goalControl.set(control) }
    }

    fun draw() {
        val truePoseCopy = getTruePose()
        val center = getPositionFromPose(truePoseCopy)
        val centerToHeadUnit = DMatrix2(cos(truePoseCopy.a3), sin(truePoseCopy.a3))
        centerToHeadUnit *= 0.5 * length
        val head = center + centerToHeadUnit
        val tail = center - centerToHeadUnit

        // Draw robot body
        applet.stroke(1)
        applet.line(center.a1.toFloat(), 0f, center.a2.toFloat(), head.a1.toFloat(), 0f, head.a2.toFloat())
        applet.circleXZ(center.a1, center.a2, length * 0.5)

        // Draw lasers
        laserSensor.draw(tail)
    }

}