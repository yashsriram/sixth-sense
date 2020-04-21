package simulator

import math.Mat2
import math.Vec2
import processing.core.PApplet

class LineSegment internal constructor(private val applet: PApplet, p1: Vec2, p2: Vec2) : Landmark() {
    private val p1: Vec2 = Vec2.of(p1)
    private val p2: Vec2 = Vec2.of(p2)
    private val length: Double = p2.minus(p1).norm()
    private val p21Normed: Vec2 = p2.minus(p1).normalizeInPlace()
    private val p21: Vec2 = p2.minus(p1)

    override fun shortestRayDistanceFrom(position: Vec2, orientation: Vec2): Double {
        val robotDirectionNormalized = orientation.normalize()
        val A = Mat2.withCols(robotDirectionNormalized.scale(-1.0), p21)
        val b = position.minus(p1)

        // If determinant is small i.e. the two directions are nearly parallel, we'll just assume no intersection
        if (Math.abs(A.determinant()) < Mat2.SINGULAR_LIMIT) {
            return (-1).toDouble()
        }
        val t1t2 = A.inverse().mult(b)
        // Check if the collision is off the ends of the line segment
        if (t1t2.y < 0 || t1t2.y > 1) {
            return (-1).toDouble()
        }
        // Check if the collision is behind the robot
        return if (t1t2.x <= 0) {
            (-1).toDouble()
        } else t1t2.x
    }

    override fun shortestDistanceFrom(position: Vec2): Double {
        val p31 = position.minus(p1)
        val projectionLength = p31.dot(p21Normed)
        return if (projectionLength < 0) {
            // Projects to before p1
            p31.norm()
        } else if (projectionLength > length) {
            // Projects to after p2
            position.minus(p2).norm()
        } else {
            // Somewhere in the middle
            p31.minus(p21Normed.scale(projectionLength)).norm()
        }
    }

    override fun draw() {
        applet.line(
                p1.x.toFloat(), 0f,
                p1.y.toFloat(),
                p2.x.toFloat(), 0f,
                p2.y.toFloat()
        )
    }
}
