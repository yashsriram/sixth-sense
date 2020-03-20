package math;

import java.util.Objects;

public class Vec2 {
    public double x, y;

    public static Vec2 zero() {
        return new Vec2(0, 0);
    }

    public static Vec2 of(double c) {
        return new Vec2(c, c);
    }

    public static Vec2 of(double x, double y) {
        return new Vec2(x, y);
    }

    public static Vec2 of(final Vec2 source) {
        return new Vec2(source.x, source.y);
    }

    public Vec2 set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        return this;
    }

    public Vec2 set(Vec2 b) {
        this.x = b.x;
        this.y = b.y;
        return this;
    }

    public Vec2 plus(Vec2 b) {
        return new Vec2(this.x + b.x, this.y + b.y);
    }

    public Vec2 plusInPlace(Vec2 b) {
        this.x += b.x;
        this.y += b.y;
        return this;
    }

    public Vec2 minus(Vec2 b) {
        return new Vec2(this.x - b.x, this.y - b.y);
    }

    public Vec2 minusInPlace(Vec2 b) {
        this.x -= b.x;
        this.y -= b.y;
        return this;
    }

    public Vec2 scale(double t) {
        return new Vec2(this.x * t, this.y * t);
    }

    public Vec2 scaleInPlace(double t) {
        this.x *= t;
        this.y *= t;
        return this;
    }

    public Vec2 normalize() {
        double abs = norm();
        if (abs < 1e-6f) {
            return new Vec2(this);
        } else {
            return new Vec2(this).scaleInPlace(1 / abs);
        }
    }

    public Vec2 normalizeInPlace() {
        double abs = norm();
        if (abs > 1e-6f) {
            scaleInPlace(1 / abs);
        }
        return this;
    }

    public double dot(Vec2 b) {
        return this.x * b.x + this.y * b.y;
    }

    public double norm() {
        return (double) Math.sqrt(this.x * this.x + this.y * this.y);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vec2)) return false;
        Vec2 vec2 = (Vec2) o;
        return Double.compare(vec2.x, x) == 0 &&
                Double.compare(vec2.y, y) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y);
    }

    @Override
    public String toString() {
        return "Vec3{" +
                "x=" + x +
                ", y=" + y +
                '}';
    }

    private Vec2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    private Vec2(Vec2 c) {
        this.x = c.x;
        this.y = c.y;
    }
}
