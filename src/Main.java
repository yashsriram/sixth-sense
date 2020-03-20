import math.Vec2;

public class Main {
    public static void main(String[] args) {
        LineSegmentFeature ls = new LineSegmentFeature(Vec2.of(-10, 0), Vec2.of(0, 0));
        System.out.println(ls.checkIntersection(Vec2.of(1, 1), Vec2.of(1, -1)));
        System.out.println(ls.shortestDistance(Vec2.of(1, 1)));
    }
}
