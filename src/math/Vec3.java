package math;

import java.util.Objects;

public class Vec3 {
    public float x, y, z;

    public static Vec3 zero() {
        return new Vec3(0, 0, 0);
    }

    public static Vec3 of(float c) {
        return new Vec3(c, c, c);
    }

    public static Vec3 of(float x, float y, float z) {
        return new Vec3(x, y, z);
    }

    public static Vec3 of(final Vec3 source) {
        return new Vec3(source.x, source.y, source.z);
    }

    public Vec3 set(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
        return this;
    }

    public Vec3 set(Vec3 b) {
        this.x = b.x;
        this.y = b.y;
        this.z = b.z;
        return this;
    }

    public Vec3 plus(Vec3 b) {
        return new Vec3(this.x + b.x, this.y + b.y, this.z + b.z);
    }

    public Vec3 plusInPlace(Vec3 b) {
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
        return this;
    }

    public Vec3 minus(Vec3 b) {
        return new Vec3(this.x - b.x, this.y - b.y, this.z - b.z);
    }

    public Vec3 minusInPlace(Vec3 b) {
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
        return this;
    }

    public Vec3 scale(float t) {
        return new Vec3(this.x * t, this.y * t, this.z * t);
    }

    public Vec3 scaleInPlace(float t) {
        this.x *= t;
        this.y *= t;
        this.z *= t;
        return this;
    }

    public Vec3 normalize() {
        float abs = norm();
        if (abs < 1e-6f) {
            return new Vec3(this);
        } else {
            return new Vec3(this).scaleInPlace(1 / abs);
        }
    }

    public Vec3 normalizeInPlace() {
        float abs = norm();
        if (abs > 1e-6f) {
            scaleInPlace(1 / abs);
        }
        return this;
    }

    public float dot(Vec3 b) {
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public Vec3 cross(Vec3 b) {
        return new Vec3(this.y * b.z - this.z * b.y, this.z * b.x - this.x * b.z, this.x * b.y - this.y * b.x);
    }

    public float norm() {
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof Vec3)) return false;
        Vec3 vec3 = (Vec3) o;
        return Float.compare(vec3.x, x) == 0 &&
                Float.compare(vec3.y, y) == 0 &&
                Float.compare(vec3.z, z) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, y, z);
    }

    @Override
    public String toString() {
        return "Vec3{" +
                "x=" + x +
                ", y=" + y +
                ", z=" + z +
                '}';
    }

    private Vec3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    private Vec3(Vec3 c) {
        this.x = c.x;
        this.y = c.y;
        this.z = c.z;
    }
}
