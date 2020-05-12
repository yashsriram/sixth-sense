package robot.sensing

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import simulator.LaserSensor
import kotlin.math.abs
import kotlin.math.pow
import kotlin.math.sqrt

class RANSACLeastSquares(private val applet: PApplet) : ObstacleLandmarkExtractor {
    companion object {
        var DISCONTINUITY_THRESHOLD = 60.0

        var RANSAC_ITER = 1000
        var RANSAC_THRESHOLD = 4f
        var RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT = 8

        var VERTICAL_LINE_THRESHOLD = 20f

        var USE_LEAST_SQUARE_FITTING = true
        var DRAW_PARTITIONS = true
        var USE_IEP = true
    }

    override fun getName(): String {
        var name = ""
        if (USE_IEP) {
            name += "IEP/"
        }
        name += "RANSAC/"
        if (USE_LEAST_SQUARE_FITTING) {
            name += "LS"
        }
        return name
    }

    private fun projectPointOnLine(m: Float, c: Float, point: FMatrix2): FMatrix2 {
        val A = FMatrix2x2(
                1 / m, 1f,
                -m, 1f
        )
        val B = FMatrix2(
                point.a2 + point.a1 / m,
                c
        )
        return A.inverse() * B
    }

    private fun leastSquaresLineSegmentFit(bestInliers: List<FMatrix2>): Pair<FMatrix2, FMatrix2> {
        // Check if it the line is perpendicular to x
        var maxX = Float.NEGATIVE_INFINITY
        var minX = Float.POSITIVE_INFINITY
        for (inlier in bestInliers) {
            if (inlier.a1 > maxX) {
                maxX = inlier.a1
            }
            if (inlier.a1 < minX) {
                minX = inlier.a1
            }
        }
        if (maxX - minX < VERTICAL_LINE_THRESHOLD) {
            val x = (maxX + minX) / 2f
            val e1 = FMatrix2(x, bestInliers.first().a2)
            val e2 = FMatrix2(x, bestInliers.last().a2)
            return Pair(e1, e2)
        } else {
            // Least squares line fitting
            val X = FMatrixRMaj(bestInliers.size, 2)
            val Y = FMatrixRMaj(bestInliers.size, 1)
            for ((t, inlier) in bestInliers.withIndex()) {
                X[t, 0] = 1f
                X[t, 1] = inlier.a1
                Y[t, 0] = inlier.a2
            }
            val X_t = X.transpose()
            val alpha = (X_t * X).inverse() * X_t * Y
            val e1 = projectPointOnLine(alpha[1], alpha[0], bestInliers.first())
            val e2 = projectPointOnLine(alpha[1], alpha[0], bestInliers.last())
            return Pair(e1, e2)
        }
    }

    private fun ransac(points: List<FMatrix2>): Pair<List<FMatrix2>, MutableList<FMatrix2>> {
        // Find the biggest line using RANSAC
        var bestInliers = mutableListOf<FMatrix2>()
        var bestRemainingPoints = mutableListOf<FMatrix2>()
        for (i in 0 until RANSAC_ITER) {
            val remainingPointsCopy = points.toMutableList()
            val definingPoints = mutableListOf(remainingPointsCopy.random(), remainingPointsCopy.random())
            // Get inlier indices
            val inlierIndices = mutableListOf<Int>()
            for ((k, possibleInlier) in remainingPointsCopy.withIndex()) {
                val dist = getPerpendicularDistance(definingPoints[0], definingPoints[1], possibleInlier)
                if (dist < RANSAC_THRESHOLD) {
                    inlierIndices.add(k)
                }
            }
            // Collect the inliers
            val inliers = mutableListOf<FMatrix2>()
            for (index in inlierIndices.reversed()) {
                inliers.add(remainingPointsCopy.removeAt(index))
            }
            // Update the best fits
            if (inlierIndices.size > bestInliers.size) {
                bestInliers = inliers
                bestRemainingPoints = remainingPointsCopy
            }
        }
        return Pair(bestInliers, bestRemainingPoints)
    }

    private fun extractLines(partition: List<FMatrix2>): List<Pair<FMatrix2, FMatrix2>> {
        val lines = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        // Make a copy of original points
        var points = partition.toMutableList()
        while (true) {
            val (inliers, remainingPoints) = ransac(points)
            // If number of inliers is big enough consider it as a line
            val prevLineCount = lines.size
            if (inliers.size > RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                if (USE_LEAST_SQUARE_FITTING) {
                    lines.add(leastSquaresLineSegmentFit(inliers))
                } else {
                    lines.add(Pair(inliers.first(), inliers.last()))
                }
                points = remainingPoints
            }
            // Exit if no new lines are found or we've run out of points
            if ((lines.size == prevLineCount) or (points.size < RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT + 2)) {
                break
            }
        }
        return lines
    }

    private fun iep(points: List<FMatrix2>, epsilon: Float): MutableList<List<FMatrix2>> {
        val resultList = mutableListOf<List<FMatrix2>>()
        var maxDistance = 0f
        var maxIndex = 0
        val end = points.size
        if (points.isNotEmpty()) {
            // Assume only one line by first and last points, and calculate max perpendicular distance to all middle points
            for (i in 1 until end - 1) {
                val perpendicularDistance = getPerpendicularDistance(points[0], points[end - 1], points[i])
                if (perpendicularDistance > maxDistance) {
                    maxDistance = perpendicularDistance
                    maxIndex = i
                }
            }
            // If a point is too far then split the line into two
            if (maxDistance > epsilon) {
                val resultList1 = iep(points.subList(0, maxIndex + 1), epsilon)
                val resultList2 = iep(points.subList(maxIndex, end), epsilon)
                resultList.addAll(resultList1)
                resultList.addAll(resultList2)
            } else {
                resultList.add(points)
            }
        }
        return resultList
    }

    private fun partitionBasedOnDiscontinuity(points: List<FMatrix2>, distances: List<Float>): Pair<List<List<FMatrix2>>, List<FMatrix2>> {
        val firstStagePartitions = mutableListOf<MutableList<Pair<FMatrix2, Int>>>()
        if (points.isEmpty()) {
            return Pair(mutableListOf(), mutableListOf())
        }
        // Find partitions using distance discontinuities
        var distanceIter = 0
        while (true) {
            if (distances[distanceIter] != LaserSensor.INVALID_DISTANCE) {
                break
            }
            distanceIter++
        }
        firstStagePartitions.add(mutableListOf(Pair(points[0], distanceIter)))
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
                firstStagePartitions.add(mutableListOf())
            }
            // Add the point to the latest segment
            firstStagePartitions[firstStagePartitions.lastIndex].add(Pair(points[i], distanceIter))
            distanceIter++
        }
        // Refine partitions using IEP and in the process find landmarks
        val landmarks = mutableListOf<FMatrix2>()
        // Detect (starting) loose end landmarks at start of first stage partitions
        for (partition in firstStagePartitions) {
            // If points less than enough to find at least one line eventually do not consider this partition for landmarks
            if (partition.size < RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                continue
            }
            val distIter = partition.first().second
            // If the distance index corresponding to partition.first is 0, do not consider these as landmarks
            if (distIter == 0) {
                continue
            }
            // If the distance corresponding to partition.first is very near to max length, do not consider it as landmark
            if (LaserSensor.MAX_DISTANCE - distances[distIter] < DISCONTINUITY_THRESHOLD) {
                continue
            }
            // If the distance corresponding to partition.first is larger prev distance, do not consider it as landmark
            if (distances[distIter] > distances[distIter - 1]) {
                continue
            }
            landmarks.add(partition.first().first)
        }
        // Detect (ending) loose end landmarks of first stage partitions
        for (partition in firstStagePartitions) {
            // If points less than enough to find at least one line eventually do not consider this partition for landmarks
            if (partition.size < RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                continue
            }
            val distIter = partition.last().second
            // If the distance index corresponding to partition.last is LaserSensor.COUNT, do not consider these as landmark
            if (distIter >= LaserSensor.COUNT - 1) {
                continue
            }
            // If the distance corresponding to partition.last is very near to max length, do not consider it as landmark
            if (LaserSensor.MAX_DISTANCE - distances[distIter] < DISCONTINUITY_THRESHOLD) {
                continue
            }
            // If the distance corresponding to partition.last is larger than next distance, do not consider it as landmark
            if (distances[distIter] > distances[distIter + 1]) {
                continue
            }
            // If all checks passed add landmark
            landmarks.add(partition.last().first)
        }
        // IEP refinement for the second stage
        if (USE_IEP) {
            val secondStagePartitions = mutableListOf<List<FMatrix2>>()
            for (partitionWithDistances in firstStagePartitions) {
                val iepPartitions = iep(partitionWithDistances.map { pointDistancePair -> pointDistancePair.first }, 15f)
                secondStagePartitions.addAll(iepPartitions)
                // Detect intersection landmarks, simpler case all such inner intersections can be considered landmarks
                // For each intersection of lines
                for (j in 1 until iepPartitions.size) {
                    val iepPartition = iepPartitions[j]
                    // If points less than enough to find at least one line eventually do not consider this partition for landmarks
                    if (iepPartition.size < RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                        continue
                    }
                    val prevIEPPartition = iepPartitions[j - 1]
                    // If two partitions are far off then there might not be an intersection
                    if (prevIEPPartition.last().minus(iepPartition.first()).norm() > RANSAC_THRESHOLD) {
                        continue
                    }
                    // If all checks passed add landmark
                    landmarks.add(iepPartition.first())
                }
            }
            return Pair(secondStagePartitions, landmarks)
        } else {
            val secondStagePartitions = mutableListOf<List<FMatrix2>>()
            for (partitionWithDistances in firstStagePartitions) {
                secondStagePartitions.add(partitionWithDistances.map { pointDistancePair -> pointDistancePair.first })
            }
            return Pair(secondStagePartitions, landmarks)
        }
    }

    override fun getObservedObstaclesAndLandmarks(inputPoints: List<FMatrix2>, distances: List<Float>): Pair<List<Pair<FMatrix2, FMatrix2>>, List<FMatrix2>> {
        val observedLineSegmentObstacles = mutableListOf<Pair<FMatrix2, FMatrix2>>()

        val (partitions, observedLandmarks) = partitionBasedOnDiscontinuity(inputPoints, distances)
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
            val linesInPartition = extractLines(partition)
            observedLineSegmentObstacles.addAll(linesInPartition)
        }

        return Pair(observedLineSegmentObstacles, observedLandmarks)
    }

    private fun getPerpendicularDistance(P1: FMatrix2, P2: FMatrix2, P0: FMatrix2): Float {
        val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
        val den = sqrt((P2.a2 - P1.a2).toDouble().pow(2.0) + (P2.a1 - P1.a1).toDouble().pow(2.0))
        return (num / den).toFloat()
    }
}
