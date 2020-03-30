package simulator.environment;

import math.Mat2;
import math.Vec2;

public class LineSegment {
    public final Vec2 p1;
    public final Vec2 p2;
    public final double length;
    public final Vec2 p21Normed;
    public final Vec2 p21;

    public LineSegment(Vec2 p1, Vec2 p2) {
        this.p1 = Vec2.of(p1);
        this.p2 = Vec2.of(p2);
        this.length = p2.minus(p1).norm();
        this.p21Normed = p2.minus(p1).normalizeInPlace();
        this.p21 = p2.minus(p1);
    }

    public double rayDistance(final Vec2 robotPosition, final Vec2 robotDirection) {
        Vec2 robotDirectionNormalized = robotDirection.normalize();
        Mat2 A = Mat2.withCols(robotDirectionNormalized.scale(-1), p21);
        Vec2 b = robotPosition.minus(p1);

        // If determinant is small i.e. the two directions are nearly parallel, we'll just assume no intersection
        if (Math.abs(A.determinant()) < Mat2.SINGULAR_LIMIT) {
            return -1;
        }
        Vec2 t1t2 = A.inverse().mult(b);
        // Check if the collision is off the ends of the line segment
        if (t1t2.y < 0 || t1t2.y > 1) {
            return -1;
        }
        // Check if the collision is behind the robot
        if (t1t2.x <= 0) {
            return -1;
        }
        return t1t2.x;
    }

    public double shortestDistance(Vec2 pose) {
        Vec2 p31 = pose.minus(p1);
        double projectionLength = p31.dot(p21Normed);
        if (projectionLength < 0) {
            // Projects to before p1
            return p31.norm();
        } else if (projectionLength > length) {
            // Projects to after p2
            return pose.minus(p2).norm();
        } else {
            // Somewhere in the middle
            return p31.minus(p21Normed.scale(projectionLength)).norm();
        }
    }

    @Override
    public String toString() {
        return "LineSegmentFeature{" +
                "p1=" + p1 +
                ", p2=" + p2 +
                '}';
    }
}
