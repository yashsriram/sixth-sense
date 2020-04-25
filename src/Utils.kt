import org.ejml.data.DMatrix2
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
    var filteredPoints = doRansacFiltering(points, 1000, 4F, 12)
    for (pointset in filteredPoints){
        observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(pointset[0], pointset[1]))
    }

    /* TODO fill "observedLandmarks" with landmarks such as intersections of line or loose ends of lines */
    val corner = DMatrix2(10.0, 0.0)
    observedLandmarks.add(corner)

    /* -- */
    return Pair(observedLineSegmentObstacles, observedLandmarks)
}

/*
* points: list of X, Y positions
* iterations: number of iterations to run RANSAC for
* threshold: max distance from the line to count as inlier
* minInliers: min number of inliers to consider a good line
* returns: MutableList<List<DMatrix2>> where each element is a list which contains the start and
* end points of the line
* */
fun doRansacFiltering(points:List<DMatrix2>, iterations: Int, threshold: Float, minInliers:Int):
        MutableList<List<DMatrix2>>{
    
    // return object
    val filteredPoints = mutableListOf<List<DMatrix2>>()

    if(points.isNotEmpty()) {
        
        // to keep track of all outliers after removal of inliers
        var outlierPoints = points.toMutableList()
        var nLines: Int

        do {
            // keep track of max inliers and remaining outliers after 
            // identification of each line
            var maxInliers = 0
            var fitPoints = listOf<DMatrix2>()
            var fitOutliers = mutableListOf<DMatrix2>()
            
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
                    fitPoints = randPoints
                    maxInliers = inliers
                    fitOutliers = outliers
                }
            }
            
            // filter out spurious matches
            nLines = filteredPoints.size
            if (maxInliers > minInliers) {
                filteredPoints.add(fitPoints)
                outlierPoints = fitOutliers
            }
            
            // exit if no new lines are found or we've run out of points
        } while ((filteredPoints.size > nLines) && (outlierPoints.size > 3))
    }
    return filteredPoints
}

/* 
* P1: First point of the line
* P2: Second point of the line
* P0: Point who's distance is to be calculated
* return: distance of P0 from the line P1P2
* */
fun getPerpendicularDistance(P1: DMatrix2, P2:DMatrix2, P0:DMatrix2):
        Float{

    val num = abs((P2.a2 - P1.a2)*P0.a1 - (P2.a1 - P1.a1)*P0.a2 + P2.a1*P1.a2  - P2.a2*P1.a1)
    val den = sqrt((P2.a2 - P1.a2).pow(2.0) + (P2.a1 - P1.a1).pow(2.0))

    return (num / den).toFloat()
}
