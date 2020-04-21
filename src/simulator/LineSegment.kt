package simulator

import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix2x2
import processing.core.PApplet
import kotlin.math.abs

class LineSegment internal constructor(private val applet: PApplet, p1: DMatrix2, p2: DMatrix2) : Landmark() {
    companion object {
        private const val SINGULAR_LIMIT = 1e-6
    }

    private val p1 = DMatrix2(p1)
    private val p2 = DMatrix2(p2)
    private val length: Double = (p2 - p1).norm()
    private val p21Normed = (p2 - p1).normalizeInPlace()
    private val p21 = (p2 - p1)

    override fun shortestRayDistanceFrom(position: DMatrix2, orientation: DMatrix2): Double {
        val firstColumn = orientation.normalize() * -1.0
        val secondColumn = p21
        val A = DMatrix2x2(firstColumn.a1, secondColumn.a1, firstColumn.a2, secondColumn.a2)
        val b = position.minus(p1)

        // If determinant is small i.e. the two directions are nearly parallel, we'll just assume no intersection
        if (abs(A.determinant()) < SINGULAR_LIMIT) {
            return -1.0
        }
        val t1t2 = A.inverse() * b
        // Check if the collision is off the ends of the line segment
        if (t1t2.a2 < 0 || t1t2.a2 > 1) {
            return (-1).toDouble()
        }
        // Check if the collision is behind the robot
        return if (t1t2.a1 <= 0) {
            -1.0
        } else {
            t1t2.a1
        }
    }

    override fun shortestDistanceFrom(position: DMatrix2): Double {
        val p31 = position - p1
        val projectionLength = p31.dot(p21Normed)
        return if (projectionLength < 0) {
            // Projects to before p1
            p31.norm()
        } else if (projectionLength > length) {
            // Projects to after p2
            (position - p2).norm()
        } else {
            // Somewhere in the middle
            (p31 - p21Normed * projectionLength).norm()
        }
    }

    override fun draw() {
        applet.line(
                p1.a1.toFloat(), 0f,
                p1.a2.toFloat(),
                p2.a1.toFloat(), 0f,
                p2.a2.toFloat()
        )
    }
}
