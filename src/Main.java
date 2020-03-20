import math.Vec2;

public class Main {
    public static void main(String[] args) {
        LineSegment ls = new LineSegment(Vec2.of(-10, 0), Vec2.of(1, 0));
        try {
            System.out.println(ls.checkIntersection(Vec2.of(1, 1), Vec2.of(1, -1)));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
