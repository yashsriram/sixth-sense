package simulator

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix3
import processing.core.PApplet
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sign
import kotlin.math.sin

class Robot internal constructor(private val applet: PApplet, val length: Float, truePose: FMatrix3, var isRunning: Boolean) {
    companion object {
        const val MAX_LINEAR_ACCELERATION = 200000f
        const val MAX_ANGULAR_ACCELERATION = 5f
        const val LINEAR_VELOCITY_ERROR_LIMIT = 2f
        const val ANGULAR_VELOCITY_ERROR_LIMIT = 0.1f
        private fun getChangeInPose(pose: FMatrix3, control: FMatrix2): FMatrix3 {
            val changeInPose = FMatrix3()
            changeInPose.a1 = control.a1 * cos(pose.a3)
            changeInPose.a2 = control.a1 * sin(pose.a3)
            changeInPose.a3 = control.a2
            return changeInPose
        }

        private fun getPositionFromPose(pose: FMatrix3): FMatrix2 {
            return FMatrix2(pose.a1, pose.a2)
        }
    }

    // Multi-thread access
    private val truePose = FMatrix3(truePose)
    private val goalControl = FMatrix2()
    private val currentControl = FMatrix2()

    // Sensors
    val laserSensor: LaserSensor = LaserSensor(applet)

    fun updatePose(dt: Float) {
        var noisyControl = FMatrix2()
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
                noisyControl = FMatrix2(currentControl)
            }
        }

        // Only apply noise if we're trying to move
        if (noisyControl.squaredNorm() != 0f) {
            noisyControl.a1 *= 1f + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT.toDouble(), LINEAR_VELOCITY_ERROR_LIMIT.toDouble()).toFloat()
            noisyControl.a2 *= 1f + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT.toDouble(), ANGULAR_VELOCITY_ERROR_LIMIT.toDouble()).toFloat()
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
            val avgChangeInPose = FMatrix3()
            avgChangeInPose += k1
            avgChangeInPose += k2 * 2f
            avgChangeInPose += k3 * 2f
            avgChangeInPose += k4
            avgChangeInPose *= dt / 6f
            truePose += avgChangeInPose
        }
    }

    fun updateSense(obstacles: List<Obstacle>) {
        // Move the center of the scanner back from the center of the robot
        val truePosition: FMatrix2
        val orientation: Float
        synchronized(truePose) {
            truePosition = getPositionFromPose(truePose)
            orientation = truePose.a3
        }
        val centerToHead = FMatrix2(cos(orientation), sin(orientation))
        centerToHead *= 0.5f * length
        val laserSource = truePosition - centerToHead
        laserSensor.updateLaserScan(laserSource, orientation, obstacles)
    }

    fun isCrashing(obstacle: Obstacle): Boolean {
        synchronized(truePose) {
            return obstacle.shortestDistanceFrom(getPositionFromPose(truePose)) < length / 2
        }
    }

    /* User callable */
    fun getTruePose(): FMatrix3 {
        val answer = FMatrix3()
        synchronized(truePose) { answer.set(truePose) }
        return answer
    }

    fun getCurrentControl(): FMatrix2 {
        val answer = FMatrix2()
        synchronized(currentControl) { answer.set(currentControl) }
        return answer
    }

    fun applyControl(control: FMatrix2) {
        synchronized(goalControl) { goalControl.set(control) }
    }

    fun draw() {
        val truePoseCopy = getTruePose()
        val center = getPositionFromPose(truePoseCopy)
        val centerToHeadUnit = FMatrix2(cos(truePoseCopy.a3), sin(truePoseCopy.a3))
        centerToHeadUnit *= 0.5f * length
        val head = center + centerToHeadUnit
        val tail = center - centerToHeadUnit

        // Draw robot body
        applet.noFill()
        applet.stroke(1)
        applet.line(center.a1, 0f, center.a2, head.a1, 0f, head.a2)
        applet.circleXZ(center.a1, center.a2, length * 0.5f)

        // Draw lasers
        laserSensor.draw(tail)
    }

}