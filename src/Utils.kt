import org.ejml.data.DMatrix2

data class ObservedLineSegment(val point1: DMatrix2, val point2: DMatrix2)
/*
* points: list of X, Y positions
* return: list of pairs of X, Y positions, each pair representing a line segment
* */
fun getInferredLines(points: List<DMatrix2>): MutableList<ObservedLineSegment> {
    val inferredLines = mutableListOf<ObservedLineSegment>()
    /* TODO fill "inferredLines" with endpoints detected after RANSAC + Least Squares, sample usage provided */
    if (points.size > 0) {
        val x0 = points[0].a1
        val y0 = points[0].a2
    }
    val point1 = DMatrix2(1.0, 0.0)
    val point2 = DMatrix2(0.0, 1.0)
    inferredLines.add(ObservedLineSegment(point1, point2))

    return inferredLines
}
