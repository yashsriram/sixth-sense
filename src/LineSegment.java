public class LineSegment {
    Vec2 p1;
    Vec2 p2;

    public LineSegment(Vec2 p1, Vec2 p2) {
        this.p1 = Vec2.of(p1);
        this.p2 = Vec2.of(p2);
    }

    double checkIntersection(Vec2 robotPosition, Vec2 robotDirection) throws Exception {
        Vec2 p21 = p2.minus(p1);
        // Make sure robotDirection is a unit vector
        robotDirection = robotDirection.normalize();

        // If the two directions are nearly parallel, we'll just assume no intersection
        if (Math.abs(robotDirection.dot(p21.normalize())) > 1.0 - 1e-5) {
            throw new Exception("No intersection possible");
        }

        Mat2 A = Mat2.withCols(robotDirection.scale(-1), p21);
        Vec2 b = robotPosition.minus(p1);

        Vec2 t1t2 = A.inverse().mult(b);
        // Check if the collision is off the ends of the line segment
        if (t1t2.y < 0 || t1t2.y > 1) {
            throw new Exception("Intersecting line outside segment");
        }
        // Check if the collision is behind the robot
        if (t1t2.x <= 0) {
            throw new Exception("Intersection behind robot");
        }
        return t1t2.x;
    }

    double distance(Vec2 p3) {
        Vec2 p21Normed = p2.minus(p1);
        double p21Norm = p21Normed.norm();
        p21Normed.normalizeInPlace();
        Vec2 p31 = p3.minus(p1);

        double projectionLength = p31.dot(p21Normed);
        if (projectionLength < 0) {
            // Projects to before p1
            return p31.norm();
        } else if (projectionLength > p21Norm) {
            // Projects to after p2
            return p3.minus(p2).norm();
        } else {
            // Somewhere in the middle
            return p31.minus(p21Normed.scale(projectionLength)).norm();
        }
    }
}
