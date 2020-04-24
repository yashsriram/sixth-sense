import org.ejml.data.DMatrix2

fun getInferredLines(points: List<DMatrix2>): MutableList<TwoPoints> {
    val inferredLines = mutableListOf<TwoPoints>()
    /* TODO fill "inferredLines" with endpoints detected after RANSAC + Least Squares */
    val point1 = DMatrix2(1.0, 0.0)
    val point2 = DMatrix2(0.0, 1.0)
    inferredLines.add(TwoPoints(point1, point2))

    return inferredLines
}
