import extensions.determinant
import extensions.dist
import extensions.inverse
import extensions.times
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import processing.core.PApplet
import simulator.LaserSensor
import kotlin.math.*

class LandmarkObstacleExtractionLogicIEP(private val applet: PApplet) {
    data class ObservedLineSegmentObstacle(val point1: FMatrix2, val point2: FMatrix2)
    companion object {
        private const val RANSAC_ITER = 1000
        private const val RANSAC_THRESHOLD = 4f
        private const val RANSAC_MIN_INLIERS_FOR_LINE_SEGMENT = 15

        private const val DISCONTINUITY_THRESHOLD = 60.0
        private const val LOWER_LANDMARK_MARGIN = 1.0
        private const val INTERSECTION_MARGIN = 30.0
    }

    fun getObservedObstaclesAndLandmarks(inputPoints: List<FMatrix2>, distances: List<Float>): Pair<MutableList<ObservedLineSegmentObstacle>, MutableList<FMatrix2>> {
        val observedLineSegmentObstacles = mutableListOf<ObservedLineSegmentObstacle>()
        val observedLandmarks = mutableListOf<FMatrix2>()

        // segment the lines into sections
        val segments = partitionBasedOnDiscontinuity(inputPoints, distances)

        // find lines in each of the segments
        for (segment in segments){

            // Do IEP and add the discovered lines to the list of our lines
            val linesInSegment = fitLineSegments(segment.toList(),10F)

            // expose all discovered lines to the visualization
            for (endPoints in linesInSegment) {
                observedLandmarks.add(endPoints.first)
                observedLandmarks.add(endPoints.second)
                observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(endPoints.first, endPoints.second))
            }
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
                if (abs(distances[i] - distances[i - 1]) > DISCONTINUITY_THRESHOLD) {
                    partitions.add(mutableListOf())
                }
                // Add the point to the latest segment
                partitions[partitions.lastIndex].add(points[i])
            }
        }
        return partitions
    }

    private fun fitLineSegments(points: List<FMatrix2>, epsilon: Float): List<Pair<FMatrix2, FMatrix2>> {

        val ResultList = mutableListOf<Pair<FMatrix2, FMatrix2>>()
        var dmax = 0.0F
        var index = 0
        val end = points.size

        if(points.isNotEmpty()) {

            for (i in 1 until end - 1) {
                val dist = getPerpendicularDistance(points[0], points[end - 1], points[i])
                if (dist > dmax) {
                    dmax = dist
                    index = i
                }
            }

            if (dmax > epsilon) {
                val ResultList1 = fitLineSegments(points.subList(0, index + 1), epsilon)
                val ResultList2 = fitLineSegments(points.subList(index, end), epsilon)
                ResultList.addAll(ResultList1)
                ResultList.addAll(ResultList2)
            } else {
                ResultList.add(Pair(points[0], points[end - 1]))
            }
        }
        return ResultList.toList()
    }

    private fun getPerpendicularDistance(P1: FMatrix2, P2: FMatrix2, P0: FMatrix2): Float {
        val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
        val den = sqrt((P2.a2 - P1.a2).toDouble().pow(2.0) + (P2.a1 - P1.a1).toDouble().pow(2.0))
        return (num / den).toFloat()
    }

}
