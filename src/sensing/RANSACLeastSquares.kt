package sensing

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import org.ejml.data.FMatrixRMaj
import processing.core.PApplet
import kotlin.math.*

class RANSACLeastSquares(private val applet: PApplet) : ObstacleLandmarkExtractor {
    companion object {
        private const val RANSAC_ITER = 1000
        private const val RANSAC_THRESHOLD = 4f
        private const val RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT = 15

        private const val DISCONTINUITY_THRESHOLD = 30.0
        private const val LOWER_LANDMARK_MARGIN = 1.0
        private const val INTERSECTION_MARGIN = 30.0
    }

    override fun getName(): String {
        return "RANSAC/LS"
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
        var maxX = 0f
        var minX = Float.POSITIVE_INFINITY
        for (inlier in bestInliers) {
            if (inlier.a1 > maxX) {
                maxX = inlier.a1
            }
            if (inlier.a1 < minX) {
                minX = inlier.a1
            }
        }
        // FIXME: use better method for lines perpendicular to x
        if (maxX - minX < 20) {
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
        //                if (prevLineCount % 2 == 0) {
//                    applet.stroke(1f, 0f, 0f)
//                } else {
//                    applet.stroke(0f, 0f, 1f)
//                }
//                for (inlier in bestInliers) {
//                    applet.circleXZ(inlier.a1, inlier.a2, 2f)
//                }
    }

    private fun extractLines(partition: List<FMatrix2>): List<Pair<FMatrix2, FMatrix2>> {
        val lines = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        // Make a copy of original points
        var remainingPoints = partition.toMutableList()
        while (true) {
            // Find the biggest line using RANSAC
            var bestInliers = mutableListOf<FMatrix2>()
            var bestRemainingPoints = mutableListOf<FMatrix2>()
            for (i in 0 until RANSAC_ITER) {
                val remainingPointsCopy = remainingPoints.toMutableList()
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
            // If number of inliers is big enough consider it as a line
            val prevLineCount = lines.size
            if (bestInliers.size > RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                lines.add(leastSquaresLineSegmentFit(bestInliers))
                remainingPoints = bestRemainingPoints
            }
            // Exit if no new lines are found or we've run out of points
            if ((lines.size == prevLineCount) or (remainingPoints.size < RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT + 2)) {
                break
            }
        }

        return lines
    }

    private fun partitionBasedOnDiscontinuity(points: List<FMatrix2>, distances: List<Float>): MutableList<MutableList<FMatrix2>> {
        val partitions = mutableListOf<MutableList<FMatrix2>>()
        if (points.isNotEmpty()) {
            // Add the first point as
            partitions.add(mutableListOf(points[0]))

            for (i in 1 until points.size) {
                // Add a new partition every time there is a significant change in distance
                if (abs(distances[i] - distances[i - 1]) > DISCONTINUITY_THRESHOLD) {
                    partitions.add(mutableListOf())
                }
                // Add the point to the latest segment
                partitions[partitions.lastIndex].add(points[i])
            }
        }
        return partitions
    }

    override fun getObservedObstaclesAndLandmarks(inputPoints: List<FMatrix2>, distances: List<Float>): Pair<List<Pair<FMatrix2, FMatrix2>>, List<FMatrix2>> {
        val observedLineSegmentObstacles = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        val observedLandmarks = mutableListOf<FMatrix2>()

        val partitions = partitionBasedOnDiscontinuity(inputPoints, distances)
        for (partition in partitions) {
            val linesInPartition = extractLines(partition)
            observedLineSegmentObstacles.addAll(linesInPartition)
        }

        // Loose ends
//        var j = 1
//        var i = 1
//        while (i < (distances.size) && j < inputPoints.size) {
//            if (((distances[i] - distances[i - 1]) > DISCONTINUITY_THRESHOLD) || (distances[i] == LaserSensor.INVALID_DISTANCE && (distances[i] - distances[i - 1]) > LOWER_LANDMARK_MARGIN)) {
//                observedLandmarks.add(inputPoints[j - 1])
//            } else if (((distances[i - 1] - distances[i]) > DISCONTINUITY_THRESHOLD) || (distances[i - 1] == LaserSensor.INVALID_DISTANCE && (distances[i - 1] - distances[i]) > LOWER_LANDMARK_MARGIN)) {
//                observedLandmarks.add(inputPoints[j])
//            }
//            if (distances[i] != LaserSensor.INVALID_DISTANCE) {
//                j++
//            }
//            i++
//        }
//
//        // Intersection
//        val intersectLandmarks = getLandmarksAtIntersection(observedLineSegmentObstacles, inputPoints)
//        for (pt in intersectLandmarks) {
//            observedLandmarks.add(pt)
//        }

        return Pair(observedLineSegmentObstacles, observedLandmarks)
    }

    private fun getPerpendicularDistance(P1: FMatrix2, P2: FMatrix2, P0: FMatrix2): Float {
        val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
        val den = sqrt((P2.a2 - P1.a2).toDouble().pow(2.0) + (P2.a1 - P1.a1).toDouble().pow(2.0))
        return (num / den).toFloat()
    }

    private fun getLandmarksAtIntersection(lines: List<Pair<FMatrix2, FMatrix2>>, points: List<FMatrix2>): MutableList<FMatrix2> {
        val observedLandmarks = mutableListOf<FMatrix2>()
        for (i in 0 until lines.size - 1) {
            for (j in i + 1 until lines.size) {
                val L1 = lines[i]
                val L2 = lines[j]

                // FIXME : what happens when den ~ 0?
                val m1 = (L1.second.a2 - L1.first.a2) / (L1.second.a1 - L1.first.a1)
                val b1 = -m1 * L1.first.a1 + L1.first.a2

                val m2 = (L2.second.a2 - L2.first.a2) / (L2.second.a1 - L2.first.a1)
                val b2 = -m2 * L2.first.a1 + L2.first.a2

                val A = FMatrix2x2(-m1, 1f, -m2, 1f)
                val b = FMatrix2(b1, b2)
                if (abs(A.determinant()) < 1e-6) {
                    continue
                }
                val intersect = A.inverse() * b
                var minDist = Float.MAX_VALUE
                var minIdx = 0
                for (p in 0 until (points.size)) {
                    if (intersect.dist(points[p]) < minDist) {
                        minIdx = p
                        minDist = intersect.dist(points[p])
                    }
                }
                if (minDist < INTERSECTION_MARGIN && checkLimit(points[minIdx], L1) && checkLimit(points[minIdx], L2)) {
                    observedLandmarks.add(points[minIdx])
                }
            }
        }
        return observedLandmarks
    }

    private fun checkLimit(pt: FMatrix2, L: Pair<FMatrix2, FMatrix2>): Boolean {
        val x1 = L.first.a1
        val y1 = L.first.a2
        val x2 = L.second.a1
        val y2 = L.second.a2
        if (pt.a1 > (min(x1, x2) - INTERSECTION_MARGIN)
                && pt.a1 < (max(x1, x2) + INTERSECTION_MARGIN)
                && pt.a2 > (min(y1, y2) - INTERSECTION_MARGIN)
                && pt.a2 < (max(y1, y2) + INTERSECTION_MARGIN)) {
            return true
        }
        return false
    }
}
