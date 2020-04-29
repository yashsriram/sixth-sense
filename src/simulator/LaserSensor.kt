package simulator

import extensions.circleXZ
import extensions.plus
import extensions.timesAssign
import org.ejml.data.DMatrix2
import processing.core.PApplet
import java.util.*
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.cos
import kotlin.math.min
import kotlin.math.sin

class LaserSensor internal constructor(private val applet: PApplet) {

    companion object {
        var DRAW_LASERS = true

        const val COUNT = 181
        const val MIN_THETA = -Math.PI / 2
        const val MAX_THETA = Math.PI / 2
        const val ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / COUNT

        const val DISTANCE_ERROR_LIMIT = 5.0
        const val ANGLE_ERROR_LIMIT = 0.05

        const val MAX_DISTANCE = 500.0
        const val INVALID_DISTANCE = MAX_DISTANCE + 1
    }

    // Multi-thread access
    private val distances: MutableList<Double> = ArrayList(COUNT)
    private val angles: MutableList<Double> = ArrayList(COUNT)

    init {
        for (i in 0 until COUNT) {
            distances.add(INVALID_DISTANCE)
            angles.add(i.toDouble())
        }
    }

    fun updateLaserScan(position: DMatrix2, orientation: Double, obstacles: List<Obstacle>) {
        val newMeasurements: MutableList<Double> = ArrayList(COUNT)
        for (i in 0 until COUNT) {
            newMeasurements.add(INVALID_DISTANCE)
        }

        // For each laser beam
        for (i in 0 until COUNT) {
            val percentage = i / (COUNT - 1.0)
            val laserThErr = ThreadLocalRandom.current().nextDouble(-ANGLE_ERROR_LIMIT * ANGULAR_RESOLUTION, ANGLE_ERROR_LIMIT * ANGULAR_RESOLUTION)
            val theta = MIN_THETA + (MAX_THETA - MIN_THETA) * percentage + orientation + laserThErr
            angles[i] = theta
            val v = DMatrix2(cos(theta), sin(theta))

            // Check intersection for each line feature
            for (landmark in obstacles) {
                val rayDistance = landmark.shortestRayDistanceFrom(position, v)
                if (rayDistance >= 0 && rayDistance < MAX_DISTANCE) {
                    newMeasurements[i] = min(rayDistance, newMeasurements[i])
                }
            }

            // Add some noise to new measurements
            if (newMeasurements[i] < INVALID_DISTANCE) {
                val laserMeasurementError = ThreadLocalRandom.current().nextDouble(-DISTANCE_ERROR_LIMIT, DISTANCE_ERROR_LIMIT)
                newMeasurements[i] += laserMeasurementError
            }
        }

        // Update measurements
        synchronized(distances) {
            for (i in newMeasurements.indices) {
                distances[i] = newMeasurements[i]
            }
        }
    }

    private fun getAngles(): List<Double> {
        var currentAngles: List<Double>
        synchronized(angles) { currentAngles = ArrayList(angles) }
        return currentAngles
    }

    /* User callable */
    fun getDistances(): List<Double> {
        var currentDistances: List<Double>
        synchronized(distances) { currentDistances = ArrayList(distances) }
        return currentDistances
    }

    fun draw(position: DMatrix2) {
        if (!DRAW_LASERS) {
            return
        }
        val distances = getDistances()
        val angles = getAngles()
        val lasersEnds: MutableList<DMatrix2> = ArrayList(COUNT)
        for (i in 0 until COUNT) {
            if (distances[i] == INVALID_DISTANCE) {
                continue
            }
            val theta = angles[i]
            val laserBeam = DMatrix2(cos(theta), sin(theta))
            laserBeam *= distances[i]
            val laserEnd = position + laserBeam
            lasersEnds.add(laserEnd)
        }
        applet.noFill()
        for (laserEnd in lasersEnds) {
            applet.stroke(1f, 0f, 0f)
            applet.line(position.a1.toFloat(), 0f, position.a2.toFloat(), laserEnd.a1.toFloat(), 0f, laserEnd.a2.toFloat())
            applet.stroke(1f, 1f, 0f)
            applet.circleXZ(laserEnd.a1, laserEnd.a2, 1.0)
        }
    }

}