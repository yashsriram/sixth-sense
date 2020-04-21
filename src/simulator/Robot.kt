package simulator

import math.Vec2
import math.Vec3
import processing.core.PApplet
import java.util.concurrent.ThreadLocalRandom

class Robot internal constructor(private val applet: PApplet, private val robotLength: Double, truePose: Vec3, var isRunning: Boolean) {
    companion object {
        var MAX_LINEAR_ACCELERATION = 20.0
        var MAX_ANGULAR_ACCELERATION = 0.5
        var LINEAR_VELOCITY_ERROR_LIMIT = 0.5
        var ANGULAR_VELOCITY_ERROR_LIMIT = 0.1
        private fun getChangeInPose(pose: Vec3, control: Vec2): Vec3 {
            val changeInPose = Vec3.zero()
            changeInPose.x = control.x * Math.cos(pose.z)
            changeInPose.y = control.x * Math.sin(pose.z)
            changeInPose.z = control.y
            return changeInPose
        }
    }

    private val truePose: Vec3 = Vec3.of(truePose)

    // Multi-thread access
    private val goalControl = Vec2.zero()
    private val currentControl = Vec2.zero()

    // Sensors
    val laser: Laser = Laser(applet)

    fun updatePose(dt: Double) {
        var controlWithNoise = Vec2.zero()
        synchronized(currentControl) {
            synchronized(goalControl) {

                // Clamp acceleration
                val controlDiff = goalControl.minus(currentControl)
                if (Math.abs(controlDiff.x) > MAX_LINEAR_ACCELERATION * dt) {
                    controlDiff.x = Math.signum(controlDiff.x) * MAX_LINEAR_ACCELERATION * dt
                }
                if (Math.abs(controlDiff.y) > MAX_ANGULAR_ACCELERATION * dt) {
                    controlDiff.y = Math.signum(controlDiff.y) * MAX_ANGULAR_ACCELERATION * dt
                }
                currentControl.plusInPlace(controlDiff)
                controlWithNoise = Vec2.of(currentControl)
            }
        }

        // Only apply noise if we're trying to move
        if (controlWithNoise.sqauredNorm() != 0.0) {
            controlWithNoise.x *= 1.0 + ThreadLocalRandom.current().nextDouble(-LINEAR_VELOCITY_ERROR_LIMIT, LINEAR_VELOCITY_ERROR_LIMIT)
            controlWithNoise.y *= 1.0 + ThreadLocalRandom.current().nextDouble(-ANGULAR_VELOCITY_ERROR_LIMIT, ANGULAR_VELOCITY_ERROR_LIMIT)
        }

        // Run the dynamics via RK4
        val k1: Vec3
        val k2: Vec3
        val k3: Vec3
        val k4: Vec3
        val x2: Vec3
        val x3: Vec3
        val x4: Vec3
        k1 = getChangeInPose(truePose, controlWithNoise)
        x2 = truePose.plus(k1.scale(0.5f * dt))
        k2 = getChangeInPose(x2, controlWithNoise)
        x3 = truePose.plus(k2.scale(0.5f * dt))
        k3 = getChangeInPose(x3, controlWithNoise)
        x4 = truePose.plus(k3.scale(dt))
        k4 = getChangeInPose(x4, controlWithNoise)
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
        val truePosition = Vec2.of(truePose.x, truePose.y)
        val laserCenter = truePosition.minus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength))
        laser.updateLaserScan(laserCenter, truePose.z, landmarks)
    }

    fun isCrashing(landmark: Landmark): Boolean {
        return landmark.shortestDistanceFrom(Vec2.of(truePose.x, truePose.y)) < robotLength
    }

    fun getCurrentControl(): Vec2 {
        val answer = Vec2.zero()
        synchronized(currentControl) { answer.set(currentControl) }
        return answer
    }

    fun applyControl(control: Vec2) {
        synchronized(goalControl) { goalControl.set(control) }
    }

    fun draw() {
        val position = Vec2.of(truePose.x, truePose.y)
        val laserEnd = position.minus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength))
        val otherEnd = position.plus(Vec2.of(Math.cos(truePose.z), Math.sin(truePose.z)).scaleInPlace(0.5 * robotLength))

        // Draw lasers
        laser.draw(laserEnd, truePose.z)

        // Draw robot body
        applet.stroke(1)
        applet.line(laserEnd.x.toFloat(), 0f, laserEnd.y.toFloat(), otherEnd.x.toFloat(), 0f, otherEnd.y.toFloat())
    }

}