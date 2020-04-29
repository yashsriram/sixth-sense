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
private val LOWER_LANDMARK_MARGIN = 1.0
private val INTERSECTION_MARGIN = 30.0

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
    val lineSegments = mutableListOf<Pair<DMatrix2, DMatrix2>>()

    // find lines in each of the segments
    for (segment in segments){

        // Do RANSAC and add the discovered lines to the list of our lines
        val linesInSegment = fitLineSegments(segment.toList(), 1000, 4f, 12)
        lineSegments.addAll(linesInSegment)

        // expose all discovered lines to the visualization
        for (endPoints in linesInSegment) {
            observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(endPoints.first, endPoints.second))
        }
    }

    /* TODO fill "observedLandmarks" with landmarks such as intersections of line or loose ends of lines */
    //loose ends
    var j =1
    var i = 1
    while ( i < (distances.size) && j<points.size){
        if(((distances[i]-distances[i-1]) > LANDMARK_MARGIN) || ( distances[i] == LaserSensor.INVALID_DISTANCE && (distances[i]-distances[i-1]) > LOWER_LANDMARK_MARGIN)) {
            observedLandmarks.add(points[j - 1])
        }
        else if(((distances[i-1]-distances[i]) > LANDMARK_MARGIN) || ( distances[i-1] == LaserSensor.INVALID_DISTANCE && (distances[i-1]-distances[i]) > LOWER_LANDMARK_MARGIN)){
            observedLandmarks.add(points[j])
        }
        if(distances[i] != LaserSensor.INVALID_DISTANCE){
            j++
        }
        i++
    }
    //intersection
    var intersectLandmarks = mutableListOf<DMatrix2>()
    intersectLandmarks = getIntersectionPoints(lineSegments, points)
    for ( pt in intersectLandmarks)
    {
        observedLandmarks.add(pt)
    }
    /* -- */


    return Pair(observedLineSegmentObstacles, observedLandmarks)
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
* lineSegments: list of line segments
* Points: list of points
* return: list of intersection points
* */
fun getIntersectionPoints(lineSegments: List<Pair<DMatrix2, DMatrix2>>, points: List<DMatrix2>): MutableList<DMatrix2>
{
    var observedLandmarks = mutableListOf<DMatrix2>()
    for (i in 0 until lineSegments.size - 1) {
        for (j in i + 1 until lineSegments.size) {
            //two points
            val L1 = lineSegments[i]
            val L2 = lineSegments[j]

            val m1 = (L1.second.a2 - L1.first.a2) / (L1.second.a1 - L1.first.a1)
            val b1 = -m1 * L1.first.a1 + L1.first.a2

            val m2 = (L2.second.a2 - L2.first.a2) / (L2.second.a1 - L2.first.a1)
            val b2 = -m2 * L2.first.a1 + L2.first.a2

            val A = DMatrix2x2(-m1, 1.0, -m2, 1.0)
            val b = DMatrix2(b1, b2)
            if (abs(A.determinant()) > 1e-6) {
                val intersect = A.inverse() * b
                var minDist = Double.MAX_VALUE
                var minIdx = 0
                for (p in 0 until (points.size)) {
                    if (dist(intersect, points[p]) < minDist) {
                        minIdx = p
                        minDist = dist(intersect, points[p])
                    }
                }
                if(minDist<30 && checkLimit(points[minIdx] ,L1.first, L1.second,  L2.first, L2.second) ) {
                    observedLandmarks.add(points[minIdx])
                }
            }
        }
    }
    return observedLandmarks
}


/*
* Pt1: First point
* Pt2: Second point
* return: distance between Pt1 and Pt2
* */
fun dist(pt1: DMatrix2, pt2: DMatrix2): Double {
    return sqrt((pt1.a1 - pt2.a1) * (pt1.a1 - pt2.a1) + (pt1.a2 - pt2.a2) * (pt1.a2 - pt2.a2))
}


/*
* Pt: A point
* p11: First point of the line1
* p12: Second point of the line1
* p21: First point of the line2
* p22: Second point of the line2
* return: true if pt lies on both the lines up to a margin
* */
fun checkLimit(pt: DMatrix2, p11: DMatrix2, p12: DMatrix2, p21: DMatrix2, p22: DMatrix2): Boolean
{

    var checkLine1 = false
    var checkLine2 = false
    var x1 = p11.a1
    var y1 = p11.a2
    var x2 = p12.a1
    var y2 = p12.a2
    if(pt.a1 > (min(x1, x2) - INTERSECTION_MARGIN) && pt.a1 < (max(x1, x2) + INTERSECTION_MARGIN) && pt.a2 > (min(y1, y2) - INTERSECTION_MARGIN) && pt.a2 < (max(y1, y2) + INTERSECTION_MARGIN)) //Line 1
        checkLine1 = true
    x1 = p21.a1
    y1 = p21.a2
    x2 = p22.a1
    y2 = p22.a2
   //
    if(pt.a1 > (min(x1, x2) - INTERSECTION_MARGIN) && pt.a1 < (max(x1, x2) + INTERSECTION_MARGIN) && pt.a2 > (min(y1, y2) - INTERSECTION_MARGIN) && pt.a2 < (max(y1, y2) + INTERSECTION_MARGIN)) //Line 2
        checkLine2 = true

    return checkLine1 && checkLine2
}


/*
* points: list of X, Y positions
* iterations: number of iterations to run RANSAC for
* threshold: max distance from the line to count as inlier
* minInliers: min number of inliers to consider a good line
* returns: MutableList<List<DMatrix2>> where each element is a list which contains the start and
* end points of the line
* */
fun fitLineSegments(points: List<DMatrix2>, iterations: Int, threshold: Float, minInliers: Int): List<Pair<DMatrix2, DMatrix2>> {
    // return object
    val lineSegments = mutableListOf<Pair<DMatrix2, DMatrix2>>()

    if (points.isNotEmpty() && points.size > 3) {
        // to keep track of all outliers after removal of inliers
        var outlierPoints = points.toMutableList()
        var nLines = 0

        while(true){
            // keep track of max inliers and remaining outliers after 
            // identification of each line
            var maxInliers = 0
            var endPoints = Pair(DMatrix2(), DMatrix2())
            var remainingPoints = mutableListOf<DMatrix2>()

            // run RANSAC to find best fit points
            for (i in 1..iterations) {
                // keep track of outliers and inliers for the points chosen
                var inliers = 0
                val outliers = outlierPoints.shuffled().toMutableList()
                val randPoints = outliers.take(2)

                // calculate number of inliers
                for (point in outlierPoints) {
                    val dist = getPerpendicularDistance(randPoints[0], randPoints[1], point)
                    if (dist < threshold) {
                        inliers += 1
                        outliers.removeAt(outliers.indexOf(point))
                    }
                }

                // update the best fits 
                if (inliers > maxInliers) {
                    endPoints = Pair(randPoints[0], randPoints[1])
                    maxInliers = inliers
                    remainingPoints = outliers
                }
            }

            // filter out spurious matches
            nLines = lineSegments.size
            if (maxInliers > minInliers) {
                lineSegments.add(endPoints)
                outlierPoints = remainingPoints
            }

            // exit if no new lines are found or we've run out of points
            if ((lineSegments.size <= nLines) or (outlierPoints.size <= 3))
                break
        }
    }

    return lineSegments
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

