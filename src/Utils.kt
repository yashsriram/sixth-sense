import org.ejml.data.DMatrix2

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

    /* -- */
    /* TODO fill "observedLineSegmentObstacles" with endpoints detected after RANSAC + Least Squares, sample usage provided */
    if (points.size > 0) {
        val x0 = points[0].a1
        val y0 = points[0].a2
    }
    val point1 = DMatrix2(1.0, 0.0)
    val point2 = DMatrix2(0.0, 1.0)
    observedLineSegmentObstacles.add(ObservedLineSegmentObstacle(point1, point2))

    /* TODO fill "observedLandmarks" with landmarks such as intersections of line or loose ends of lines */
    val corner = DMatrix2(0.0, 0.0)
    observedLandmarks.add(corner)

    /* -- */
    return Pair(observedLineSegmentObstacles, observedLandmarks)
}
