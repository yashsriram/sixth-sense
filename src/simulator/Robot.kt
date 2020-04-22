package simulator

import extensions.*
import math.Vec3
import org.ejml.data.DMatrix2
import processing.core.PApplet
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.cos
import kotlin.math.sin

class Robot internal constructor(private val applet: PApplet, private val robotLength: Double, truePose: Vec3, var isRunning: Boolean) {
    companion object {
        var MAX_LINEAR_ACCELERATION = 20.0
        var MAX_ANGULAR_ACCELERATION = 0.5
        var LINEAR_VELOCITY_ERROR_LIMIT = 0.5
        var ANGULAR_VELOCITY_ERROR_LIMIT = 0.1
        private fun getChangeInPose(pose: Vec3, control: DMatrix2): Vec3 {
            val changeInPose = Vec3.zero()
            changeInPose.x = control.a1 * Math.cos(pose.z)
            changeInPose.y = control.a1 * Math.sin(pose.z)
            changeInPose.z = control.a2
            return changeInPose
        }
    }

    private val truePose: Vec3 = Vec3.of(truePose)

    // Multi-thread access
    private val goalControl = DMatrix2()
    private val currentControl = DMatrix2()

    // Sensors
    val laser: Laser = Laser(applet)

    fun updatePose(dt: Double) {
        var controlWithNoise = DMatrix2()
        synchronized(currentControl) {
            synchronized(goalControl) {

                // Clamp acceleration
                val controlDiff = goalControl - currentControl
                if (Math.abs(controlDiff.a1) > MAX_LINEAR_ACCELERATION * dt) {
                    controlDiff.a1 = Math.signum(controlDiff.a1) * MAX_LINEAR_ACCELERATION * dt
                }
                if (Math.abs(controlDiff.a2) > MAX_ANGULAR_ACCELERATION * dt) {
                    controlDiff.a2 = Math.signum(controlDiff.a2) * MAX_ANGULAR_ACCELERATION * dt
                }
                currentControl += controlDiff
                controlWithNoise = DMatrix2(currentControl)
            }
        }

        // Only apply noise if we're trying to move
        if (controlWithNoise.squaredNorm() != 0.0) {
            controlWithNoise.a1 *= 1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT)
            controlWithNoise.a2 *= 1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT)
        }

        // Run the dynamics via RK4
        val k1 = getChangeInPose(truePose, controlWithNoise)
        val x2 = truePose.plus(k1.scale(0.5f * dt))
        val k2 = getChangeInPose(x2, controlWithNoise)
        val x3 = truePose.plus(k2.scale(0.5f * dt))
        val k3 = getChangeInPose(x3, controlWithNoise)
        val x4 = truePose.plus(k3.scale(dt))
        val k4 = getChangeInPose(x4, controlWithNoise)
        val avergeChangeInPose = Vec3.zero()
        avergeChangeInPose
                .plusInPlace(k1)
                .plusInPlace(k2.scale(2.0))
                .plusInPlace(k3.scale(2.0))
                .plusInPlace(k4)
                .scaleInPlace(dt / 6.0)
        truePose.plusInPlace(avergeChangeInPose)
    }

    fun updateSense(landmarks: List<Landmark>) {
        // Move the center of the scanner back from the center of the robot
        val truePosition = DMatrix2(truePose.x, truePose.y)
        val centerToHead = DMatrix2(cos(truePose.z), sin(truePose.z))
        centerToHead *= 0.5 * robotLength
        val laserCenter = truePosition - centerToHead
        laser.updateLaserScan(laserCenter, truePose.z, landmarks)
    }

    fun isCrashing(landmark: Landmark): Boolean {
        return landmark.shortestDistanceFrom(DMatrix2(truePose.x, truePose.y)) < robotLength
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
        val position = DMatrix2(truePose.x, truePose.y)
        val centerToHead = DMatrix2(cos(truePose.z), sin(truePose.z))
        centerToHead *= 0.5 * robotLength
        val head = position + centerToHead
        val tail = position - centerToHead

        // Draw lasers
        laser.draw(tail, truePose.z)

        // Draw robot body
        applet.stroke(1)
        applet.line(tail.a1.toFloat(), 0f, tail.a2.toFloat(), head.a1.toFloat(), 0f, head.a2.toFloat())
    }

}