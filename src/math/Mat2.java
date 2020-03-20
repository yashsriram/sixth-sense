package math;

import java.util.Objects;

public class Mat2 {
    public static double SINGULAR_LIMIT = 1e-6;
    double a, b;
    double c, d;

    public static Mat2 withCols(Vec2 col1, Vec2 col2) {
        return new Mat2(col1.x, col2.x, col1.y, col2.y);
    }

    public static Mat2 of(double a, double b, double c, double d) {
        return new Mat2(a, b, c, d);
    }

    public static Mat2 identity() {
        return new Mat2(1, 0, 0, 1);
    }

    public Mat2 scale(double s) {
        return new Mat2(a * s, b * s, c * s, d * s);
    }

    public Mat2 scaleInPlace(double s) {
        a = a * s;
        b = b * s;
        c = c * s;
        d = d * s;
        return this;
    }

    public Mat2 plus(Mat2 other) {
        return new Mat2(a + other.a, b + other.b, c + other.c, d + other.d);
    }

    public Mat2 plusInPlace(Mat2 other) {
        a = a + other.a;
        b = b + other.b;
        c = c + other.c;
        d = d + other.d;
        return this;
    }

    public Mat2 minus(Mat2 other) {
        return new Mat2(a - other.a, b - other.b, c - other.c, d - other.d);
    }

    public Mat2 minusInPlace(Mat2 other) {
        a = a - other.a;
        b = b - other.b;
        c = c - other.c;
        d = d - other.d;
        return this;
    }

    public double determinant() {
        return a * d - b * c;
    }

    public Mat2 inverse() throws Exception {
        double determinant = determinant();
        if (Math.abs(determinant) < SINGULAR_LIMIT) {
            throw new Exception("Inverse of a singular 2x2 matrix attempted");
        }
        return new Mat2(d, -b, -c, a).scaleInPlace(1 / determinant);
    }

    public Vec2 mult(Vec2 vec2) {
        return Vec2.of(a * vec2.x + b * vec2.y, c * vec2.x + d * vec2.y);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Mat2)) return false;
        Mat2 mat2 = (Mat2) o;
        return Double.compare(mat2.a, a) == 0 &&
                Double.compare(mat2.b, b) == 0 &&
                Double.compare(mat2.c, c) == 0 &&
                Double.compare(mat2.d, d) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(a, b, c, d);
    }

    @Override
    public String toString() {
        return "math.Mat2\n" +
                "["
                + a + ", " + b +
                "\n"
                + c + ", " + d +
                "]";
    }

    private Mat2(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }
}
