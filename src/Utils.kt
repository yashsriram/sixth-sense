import extensions.determinant
import extensions.inverse
import extensions.times
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix2x2
import simulator.LaserSensor
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.*


data class ObservedLineSegmentObstacle(val point1: DMatrix2, val point2: DMatrix2)

private val LANDMARK_MARGIN = 60.0

/*
* points: list of X, Y positions
* return: pair
*   in which 1st element is list of pairs of X, Y positions, each pair representing a line segment
*        and 2nd element is list of landmarks, for us they are just intersections or ends of open line segments
* */
fun getObservedObstaclesAndLandmarks(points: List<DMatrix2>, distances: List<Double>):
        Pair<MutableList<ObservedLineSegmentObstacle>, MutableList<DMatrix2>> {
    val observedLineSegmentObstacles = mutableListOf<ObservedLineSegmentObstacle>()
    val observedLandmarks = mutableListOf<DMatrix2>()

    // segment the lines into sections
    val segments = segmentPoints(points, distances)

    // find lines in each of the segments
    for (segment in segments){

        // Do RANSAC and add the discovered lines to the list of our lines
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

/*
* points: list of input points to fit the algorithm over
* epsilon: distance margin to consider
* return: the end points of each line segment
*/
fun fitLineSegments(points: List<DMatrix2>, epsilon: Float): List<Pair<DMatrix2, DMatrix2>> {

    val ResultList = mutableListOf<Pair<DMatrix2, DMatrix2>>()
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


/*
* points: list of X, Y positions
* distances: list of distances to the points
* return List of Mutuable list of points where each mutable list corresponds to one
* segment of the distances
* */
fun segmentPoints(points: List<DMatrix2>, distances: List<Double>): List<MutableList<DMatrix2>> {

    val segments = mutableListOf<MutableList<DMatrix2>>()

    if(points.isNotEmpty()){

        // add the first segment
        segments.add(mutableListOf<DMatrix2>())
        segments.get(segments.lastIndex).add(points[0])

        for (i in 1 until points.size){

            // add a new segment everytime there is a significant change in distance
            if(Math.abs(distances[i] - distances[i-1]) > LANDMARK_MARGIN){
                segments.add(mutableListOf<DMatrix2>())
            }
            // add the point to the latest segment
            segments.get(segments.lastIndex).add(points[i])
        }

    }
    return segments.toList()
}


/* 
* P1: First point of the line
* P2: Second point of the line
* P0: Point who's distance is to be calculated
* return: distance of P0 from the line P1P2
* */
fun getPerpendicularDistance(P1: DMatrix2, P2: DMatrix2, P0: DMatrix2): Float {
    val num = abs((P2.a2 - P1.a2) * P0.a1 - (P2.a1 - P1.a1) * P0.a2 + P2.a1 * P1.a2 - P2.a2 * P1.a1)
    val den = sqrt((P2.a2 - P1.a2).pow(2.0) + (P2.a1 - P1.a1).pow(2.0))
    return (num / den).toFloat()
}

