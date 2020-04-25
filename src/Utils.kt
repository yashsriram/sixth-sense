import extensions.determinant
import extensions.inverse
import extensions.times
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix2x2
import simulator.LaserSensor
import kotlin.math.abs
import kotlin.math.sqrt
import kotlin.math.pow

data class ObservedLineSegmentObstacle(val point1: DMatrix2, val point2: DMatrix2)

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

    // Do RANSAC and add the filtered points
    val lineSegments = fitLineSegments(points, 1000, 4f, 12)
    for (endPoints in lineSegments) {
        observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(endPoints.first, endPoints.second))
    }

    /* TODO fill "observedLandmarks" with landmarks such as intersections of line or loose ends of lines */
    //loose ends
    val thr = 50.0
    for (i in 1 until (points.size)) {
        if (distances[i] == LaserSensor.INVALID_MEASUREMENT || distances[i] - distances[i - 1] > thr) {
            observedLandmarks.add(points[i - 1])
        } else if (distances[i - 1] == LaserSensor.INVALID_MEASUREMENT || distances[i - 1] - distances[i] > thr) {
            observedLandmarks.add(points[i])
        }
    }
    //intersection
    for (i in 0 until lineSegments.size - 1) {
        for (j in i + 1 until lineSegments.size) {
            //two points
            //print(i)
            //print(j)
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
                if (minDist < 30) {
                    observedLandmarks.add(points[minIdx])
                }
            }
        }
    }
    /* -- */
    return Pair(observedLineSegmentObstacles, observedLandmarks)
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

    if (points.isNotEmpty()) {
        // to keep track of all outliers after removal of inliers
        var outlierPoints = points.toMutableList()
        var nLines: Int

        do {
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
        } while ((lineSegments.size > nLines) && (outlierPoints.size > 3))
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
