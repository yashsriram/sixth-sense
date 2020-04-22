package simulator

import extensions.plus
import extensions.timesAssign
import org.ejml.data.DMatrix2
import processing.core.PApplet
import java.util.*
import java.util.concurrent.ThreadLocalRandom
import kotlin.math.cos
import kotlin.math.sin

class Laser internal constructor(private val applet: PApplet) {

    companion object {
        const val COUNT = 181

        // Angle
        const val MIN_THETA = -Math.PI / 2
        const val MAX_THETA = Math.PI / 2
        const val ANGULAR_RESOLUTION = (MAX_THETA - MIN_THETA) / COUNT
        var ANGLE_ERROR_LIMIT = 0.05

        // Distance
        var MAX_DISTANCE = 500.0
        var DISTANCE_ERROR_LIMIT = 0.05
        private fun invalidMeasurementValue(): Double {
            return MAX_DISTANCE + 1
        }
    }

    // Multi-thread access
    private val measurements: MutableList<Double> = ArrayList(COUNT)

    init {
        for (i in 0 until COUNT) {
            measurements.add(invalidMeasurementValue())
        }
    }

    fun updateLaserScan(position: DMatrix2, orientation: Double, landmarks: List<Landmark>) {
        val newMeasurements: MutableList<Double> = ArrayList(COUNT)
        for (i in 0 until COUNT) {
            newMeasurements.add(invalidMeasurementValue())
        }

        // For each laser beam
        for (i in 0 until COUNT) {
            val percentage = i / (COUNT - 1.0)
            val laserThErr = ThreadLocalRandom.current().nextDouble(-ANGLE_ERROR_LIMIT * ANGULAR_RESOLUTION, ANGLE_ERROR_LIMIT * ANGULAR_RESOLUTION)
            val theta = MIN_THETA + (MAX_THETA - MIN_THETA) * percentage + orientation + laserThErr
            val v = DMatrix2(cos(theta), sin(theta))

            // Check intersection for each line feature
            for (landmark in landmarks) {
                val rayDistance = landmark.shortestRayDistanceFrom(position, v)
                if (rayDistance >= 0 && rayDistance < MAX_DISTANCE) {
                    newMeasurements[i] = Math.min(rayDistance, newMeasurements[i])
                }
            }

            // Add some noise to new measurements
            if (newMeasurements[i] < invalidMeasurementValue()) {
                val laser_d_err = ThreadLocalRandom.current().nextDouble(-DISTANCE_ERROR_LIMIT, DISTANCE_ERROR_LIMIT)
                newMeasurements[i] = newMeasurements[i] + laser_d_err
            }
        }

        // Update measurements
        synchronized(measurements) {
            for (i in newMeasurements.indices) {
                measurements[i] = newMeasurements[i]
            }
        }
    }

    fun getMeasurements(): List<Double> {
        var currentMeasurements: List<Double>
        synchronized(measurements) { currentMeasurements = ArrayList(measurements) }
        return currentMeasurements
    }

    fun draw(position: DMatrix2, orientation: Double) {
        val distances = getMeasurements()
        val lasers: MutableList<DMatrix2> = ArrayList(COUNT)
        for (i in distances.indices) {
            if (distances[i] == invalidMeasurementValue()) {
                continue
            }
            val percentage = i / (COUNT - 1.0)
            val theta = MIN_THETA + (MAX_THETA - MIN_THETA) * percentage
            val laserBeam = DMatrix2(cos(orientation + theta),
                    sin(orientation + theta))
            laserBeam *= distances[i]
            val laserEnd = position + laserBeam
            lasers.add(laserEnd)
        }
        applet.stroke(1f, 0f, 0f)
        for (l in lasers) {
            applet.line(position.a1.toFloat(), 0f, position.a2.toFloat(), l.a1.toFloat(), 0f, l.a2.toFloat())
        }
    }

}