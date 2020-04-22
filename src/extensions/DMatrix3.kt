package extensions

import org.ejml.data.DMatrix3
import org.ejml.dense.fixed.CommonOps_DDF3
import kotlin.math.sqrt

operator fun DMatrix3.plus(b: DMatrix3): DMatrix3 {
    val sum = DMatrix3()
    CommonOps_DDF3.add(this, b, sum)
    return sum
}

operator fun DMatrix3.minus(b: DMatrix3): DMatrix3 {
    val sum = DMatrix3()
    CommonOps_DDF3.subtract(this, b, sum)
    return sum
}

operator fun DMatrix3.times(t: Double): DMatrix3 {
    return DMatrix3(a1 * t, a2 * t, a3 * t)
}

operator fun DMatrix3.plusAssign(b: DMatrix3) {
    CommonOps_DDF3.addEquals(this, b)
}

operator fun DMatrix3.minusAssign(b: DMatrix3) {
    CommonOps_DDF3.subtractEquals(this, b)
}

operator fun DMatrix3.timesAssign(t: Double) {
    this.a1 *= t
    this.a2 *= t
    this.a3 *= t
}

fun DMatrix3.dot(b: DMatrix3): Double {
    return CommonOps_DDF3.dot(this, b)
}

fun DMatrix3.norm(): Double {
    return sqrt(this.a1 * this.a1 + this.a2 * this.a2 + this.a3 * this.a3)
}

fun DMatrix3.squaredNorm(): Double {
    return this.a1 * this.a1 + this.a2 * this.a2 + this.a3 * this.a3
}

fun DMatrix3.normalize(): DMatrix3 {
    val abs = norm()
    if (abs > 1e-6f) {
        val normed = DMatrix3(this)
        normed *= (1 / abs)
        return normed
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun DMatrix3.normalizeInPlace(): DMatrix3 {
    val norm = this.norm()
    if (norm > 1e-6f) {
        this *= (1 / norm)
        return this
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun DMatrix3.prettyPrint(): String {
    return "Dense 64-bit float Mat 3 x 1 =\n${a1}\n${a2}\n${a3}"
}
