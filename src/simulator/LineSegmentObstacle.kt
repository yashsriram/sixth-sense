package simulator

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import processing.core.PApplet
import kotlin.math.abs

class LineSegmentObstacle internal constructor(private val applet: PApplet, p1: FMatrix2, p2: FMatrix2) : Obstacle() {
    companion object {
        private const val SINGULAR_LIMIT = 1e-6
    }

    private val p1 = FMatrix2(p1)
    private val p2 = FMatrix2(p2)
    private val length: Float = (p2 - p1).norm()
    private val p21Normed = (p2 - p1).normalizeInPlace()
    private val p21 = (p2 - p1)

    override fun shortestRayDistanceFrom(position: FMatrix2, orientation: FMatrix2): Float {
        val firstColumn = orientation.normalize() * -1f
        val secondColumn = p21
        val A = FMatrix2x2(firstColumn.a1, secondColumn.a1, firstColumn.a2, secondColumn.a2)
        val b = position.minus(p1)

        // If determinant is small i.e. the two directions are nearly parallel, we'll just assume no intersection
        if (abs(A.determinant()) < SINGULAR_LIMIT) {
            return -1f
        }
        val t1t2 = A.inverse() * b
        // Check if the collision is off the ends of the line segment
        if (t1t2.a2 < 0 || t1t2.a2 > 1) {
            return (-1).toFloat()
        }
        // Check if the collision is behind the robot
        return if (t1t2.a1 <= 0) {
            -1f
        } else {
            t1t2.a1
        }
    }

    override fun shortestDistanceFrom(position: FMatrix2): Float {
        val p31 = position - p1
        val projectionLength = p31.dot(p21Normed)
        return when {
            projectionLength < 0 -> {
                // Projects to before p1
                p31.norm()
            }
            projectionLength > length -> {
                // Projects to after p2
                (position - p2).norm()
            }
            else -> {
                // Somewhere in the middle
                (p31 - p21Normed * projectionLength).norm()
            }
        }
    }

    override fun draw() {
        applet.stroke(1)
        applet.line(
                p1.a1, 0f,
                p1.a2,
                p2.a1, 0f,
                p2.a2
        )
    }
}
