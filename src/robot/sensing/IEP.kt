package robot.sensing

import extensions.circleXZ
import org.ejml.data.FMatrix2
import processing.core.PApplet
import simulator.LaserSensor
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class IEP(private val applet: PApplet) : ObstacleLandmarkExtractor {
    companion object {
        private const val DISCONTINUITY_THRESHOLD = 20.0
        var DRAW_PARTITIONS = true
    }

    override fun getName(): String {
        return "IEP"
    }

    private fun getPerpendicularDistance(P1: FMatrix2, P2: FMatrix2, P0: FMatrix2): Float {
        val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
        val den = sqrt((P2.a2 - P1.a2).toDouble().pow(2.0) + (P2.a1 - P1.a1).toDouble().pow(2.0))
        return (num / den).toFloat()
    }

    private fun fitLineSegments(points: List<FMatrix2>, epsilon: Float): List<Pair<FMatrix2, FMatrix2>> {
        val resultList = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        var maxDistance = 0.0F
        var maxIndex = 0
        val end = points.size
        if (points.isNotEmpty()) {
            // Assume only one line
            for (i in 1 until end - 1) {
                val perpendicularDistance = getPerpendicularDistance(points[0], points[end - 1], points[i])
                if (perpendicularDistance > maxDistance) {
                    maxDistance = perpendicularDistance
                    maxIndex = i
                }
            }
            // If a point is too far then split the line into two
            if (maxDistance > epsilon) {
                val resultList1 = fitLineSegments(points.subList(0, maxIndex + 1), epsilon)
                val resultList2 = fitLineSegments(points.subList(maxIndex, end), epsilon)
                resultList.addAll(resultList1)
                resultList.addAll(resultList2)
            } else {
                resultList.add(Pair(points[0], points[end - 1]))
            }
        }
        return resultList.toList()
    }

    private fun partitionBasedOnDiscontinuity(points: List<FMatrix2>, distances: List<Float>): MutableList<MutableList<FMatrix2>> {
        val partitions = mutableListOf<MutableList<FMatrix2>>()
        if (points.isNotEmpty()) {
            // Add the first point as
            var distanceIter = 0
            while (true) {
                if (distances[distanceIter] != LaserSensor.INVALID_DISTANCE) {
                    break
                }
                distanceIter++
            }
            partitions.add(mutableListOf(points[0]))
            distanceIter++
            for (i in 1 until points.size) {
                while (true) {
                    if (distances[distanceIter] != LaserSensor.INVALID_DISTANCE) {
                        break
                    }
                    distanceIter++
                }
                // Add a new partition every time there is a significant change in distance
                if (abs(distances[distanceIter] - distances[distanceIter - 1]) > DISCONTINUITY_THRESHOLD) {
                    partitions.add(mutableListOf())
                }
                // Add the point to the latest segment
                partitions[partitions.lastIndex].add(points[i])
                distanceIter++
            }
        }
        return partitions
    }

    override fun getObservedObstaclesAndLandmarks(inputPoints: List<FMatrix2>, distances: List<Float>): Pair<MutableList<Pair<FMatrix2, FMatrix2>>, MutableList<FMatrix2>> {
        val observedLineSegmentObstacles = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        val observedLandmarks = mutableListOf<FMatrix2>()
        val partitions = partitionBasedOnDiscontinuity(inputPoints, distances)
        if (DRAW_PARTITIONS) {
            print("#partitions=${partitions.size}\r")
            for ((i, partition) in partitions.withIndex()) {
                when (i % 6) {
                    0 -> applet.stroke(1f, 0f, 0f)
                    1 -> applet.stroke(1f, 1f, 0f)
                    2 -> applet.stroke(0f, 1f, 0f)
                    3 -> applet.stroke(0f, 1f, 1f)
                    4 -> applet.stroke(0f, 0f, 1f)
                    5 -> applet.stroke(1f, 0f, 1f)
                }
                for (point in partition) {
                    applet.circleXZ(point.a1, point.a2, 2f)
                }
            }
        }
        for (partition in partitions) {
            val linesSegmentsInPartition = fitLineSegments(partition.toList(), 10F)
            for (endPoints in linesSegmentsInPartition) {
                observedLandmarks.add(endPoints.first)
                observedLandmarks.add(endPoints.second)
                observedLineSegmentObstacles.add(endPoints)
            }
        }
        return Pair(observedLineSegmentObstacles, observedLandmarks)
    }
}
