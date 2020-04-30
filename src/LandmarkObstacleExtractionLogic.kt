import extensions.determinant
import extensions.dist
import extensions.inverse
import extensions.times
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import simulator.LaserSensor
import kotlin.math.*

class LandmarkObstacleExtractionLogic {
    data class ObservedLineSegmentObstacle(val point1: FMatrix2, val point2: FMatrix2)
    companion object {
        private const val RANSAC_ITER = 1000
        private const val RANSAC_THRESHOLD = 4f
        private const val RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT = 15

        private const val LANDMARK_MARGIN = 60.0
        private const val LOWER_LANDMARK_MARGIN = 1.0
        private const val INTERSECTION_MARGIN = 30.0

        fun getObservedObstaclesAndLandmarks(points: List<FMatrix2>, distances: List<Float>): Pair<MutableList<ObservedLineSegmentObstacle>, MutableList<FMatrix2>> {
            val observedLineSegmentObstacles = mutableListOf<ObservedLineSegmentObstacle>()
            val observedLandmarks = mutableListOf<FMatrix2>()

            // Partition points based on distance discontinuities
            val partitions = partitionBasedOnDiscontinuity(points, distances)
            val lines = mutableListOf<Pair<FMatrix2, FMatrix2>>()
            // Find lines in each of the segments
            for (partition in partitions) {
                // Do RANSAC and add the discovered line to the list of our lines
                val linesInPartition = fitLines(partition)
                lines.addAll(linesInPartition)
            }
            // Expose all discovered lines to the visualization
            // FIXME: return the line segment (using intersections of loose ends) not the defining points of the line
            for (endPoints in lines) {
                observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(endPoints.first, endPoints.second))
            }

            // Loose ends
            var j = 1
            var i = 1
            while (i < (distances.size) && j < points.size) {
                if (((distances[i] - distances[i - 1]) > LANDMARK_MARGIN) || (distances[i] == LaserSensor.INVALID_DISTANCE && (distances[i] - distances[i - 1]) > LOWER_LANDMARK_MARGIN)) {
                    observedLandmarks.add(points[j - 1])
                } else if (((distances[i - 1] - distances[i]) > LANDMARK_MARGIN) || (distances[i - 1] == LaserSensor.INVALID_DISTANCE && (distances[i - 1] - distances[i]) > LOWER_LANDMARK_MARGIN)) {
                    observedLandmarks.add(points[j])
                }
                if (distances[i] != LaserSensor.INVALID_DISTANCE) {
                    j++
                }
                i++
            }

            // Intersection
            val intersectLandmarks = getIntersectionPoints(lines, points)
            for (pt in intersectLandmarks) {
                observedLandmarks.add(pt)
            }

            return Pair(observedLineSegmentObstacles, observedLandmarks)
        }

        private fun partitionBasedOnDiscontinuity(points: List<FMatrix2>, distances: List<Float>): MutableList<MutableList<FMatrix2>> {
            val partitions = mutableListOf<MutableList<FMatrix2>>()
            if (points.isNotEmpty()) {
                // Add the first point as
                partitions.add(mutableListOf(points[0]))

                for (i in 1 until points.size) {
                    // Add a new partition every time there is a significant change in distance
                    if (abs(distances[i] - distances[i - 1]) > LANDMARK_MARGIN) {
                        partitions.add(mutableListOf())
                    }
                    // Add the point to the latest segment
                    partitions[partitions.lastIndex].add(points[i])
                }
            }
            return partitions
        }

        private fun fitLines(points: List<FMatrix2>): List<Pair<FMatrix2, FMatrix2>> {
            val lineSegments = mutableListOf<Pair<FMatrix2, FMatrix2>>()

            if (points.isNotEmpty() && points.size > 3) {
                // To keep track of all outliers after removal of inliers
                var outlierPoints = points.toMutableList()

                while (true) {
                    // Keep track of max inliers and remaining outliers after
                    // Identification of each line
                    var maxInliers = 0
                    var endPoints = Pair(FMatrix2(), FMatrix2())
                    var remainingPoints = mutableListOf<FMatrix2>()

                    // Run RANSAC to find best fit points
                    for (i in 1..RANSAC_ITER) {
                        // Keep track of outliers and inliers for the points chosen
                        val outliersShuffledCopy = outlierPoints.shuffled().toMutableList()
                        val randPoints = outliersShuffledCopy.take(2)
                        // Calculate number of inliers
                        var inliers = 0
                        for (point in outlierPoints) {
                            val dist = getPerpendicularDistance(randPoints[0], randPoints[1], point)
                            if (dist < RANSAC_THRESHOLD) {
                                inliers += 1
                                outliersShuffledCopy.removeAt(outliersShuffledCopy.indexOf(point))
                            }
                        }
                        // Update the best fits
                        if (inliers > maxInliers) {
                            endPoints = Pair(randPoints[0], randPoints[1])
                            maxInliers = inliers
                            remainingPoints = outliersShuffledCopy
                        }
                    }

                    // Filter out spurious matches
                    val prevLineCount = lineSegments.size
                    if (maxInliers > RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT) {
                        lineSegments.add(endPoints)
                        outlierPoints = remainingPoints
                    }

                    // Exit if no new lines are found or we've run out of points
                    if ((lineSegments.size <= prevLineCount) or (outlierPoints.size <= 3)) {
                        break
                    }
                }
            }

            return lineSegments
        }

        private fun getIntersectionPoints(lineSegments: List<Pair<FMatrix2, FMatrix2>>, points: List<FMatrix2>): MutableList<FMatrix2> {
            var observedLandmarks = mutableListOf<FMatrix2>()
            for (i in 0 until lineSegments.size - 1) {
                for (j in i + 1 until lineSegments.size) {
                    //two points
                    val L1 = lineSegments[i]
                    val L2 = lineSegments[j]

                    val m1 = (L1.second.a2 - L1.first.a2) / (L1.second.a1 - L1.first.a1)
                    val b1 = -m1 * L1.first.a1 + L1.first.a2

                    val m2 = (L2.second.a2 - L2.first.a2) / (L2.second.a1 - L2.first.a1)
                    val b2 = -m2 * L2.first.a1 + L2.first.a2

                    val A = FMatrix2x2(-m1, 1f, -m2, 1f)
                    val b = FMatrix2(b1, b2)
                    if (abs(A.determinant()) > 1e-6) {
                        val intersect = A.inverse() * b
                        var minDist = Float.MAX_VALUE
                        var minIdx = 0
                        for (p in 0 until (points.size)) {
                            if (intersect.dist(points[p]) < minDist) {
                                minIdx = p
                                minDist = intersect.dist(points[p])
                            }
                        }
                        if (minDist < 30 && checkLimit(points[minIdx], L1.first, L1.second, L2.first, L2.second)) {
                            observedLandmarks.add(points[minIdx])
                        }
                    }
                }
            }
            return observedLandmarks
        }

        /*
        * Pt: A point
        * p11: First point of the line1
        * p12: Second point of the line1
        * p21: First point of the line2
        * p22: Second point of the line2
        * return: true if pt lies on both the lines up to a margin
        * */
        private fun checkLimit(pt: FMatrix2, p11: FMatrix2, p12: FMatrix2, p21: FMatrix2, p22: FMatrix2): Boolean {
            var checkLine1 = false
            var checkLine2 = false
            var x1 = p11.a1
            var y1 = p11.a2
            var x2 = p12.a1
            var y2 = p12.a2
            // Line 1
            if (pt.a1 > (min(x1, x2) - INTERSECTION_MARGIN) && pt.a1 < (max(x1, x2) + INTERSECTION_MARGIN) && pt.a2 > (min(y1, y2) - INTERSECTION_MARGIN) && pt.a2 < (max(y1, y2) + INTERSECTION_MARGIN)) {
                checkLine1 = true
            }
            x1 = p21.a1
            y1 = p21.a2
            x2 = p22.a1
            y2 = p22.a2
            // Line 2
            if (pt.a1 > (min(x1, x2) - INTERSECTION_MARGIN) && pt.a1 < (max(x1, x2) + INTERSECTION_MARGIN) && pt.a2 > (min(y1, y2) - INTERSECTION_MARGIN) && pt.a2 < (max(y1, y2) + INTERSECTION_MARGIN)) {
                checkLine2 = true
            }
            return checkLine1 && checkLine2
        }

        /*
        * P1: First point of the line
        * P2: Second point of the line
        * P0: Point who's distance is to be calculated
        * return: distance of P0 from the line P1P2
        * */
        private fun getPerpendicularDistance(P1: FMatrix2, P2: FMatrix2, P0: FMatrix2): Float {
            val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
            val den = sqrt((P2.a2 - P1.a2).toDouble().pow(2.0) + (P2.a1 - P1.a1).toDouble().pow(2.0))
            return (num / den).toFloat()
        }

    }
}

