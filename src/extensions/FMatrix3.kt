package extensions

import org.ejml.data.FMatrix3
import org.ejml.dense.fixed.CommonOps_FDF3
import kotlin.math.sqrt

operator fun FMatrix3.plus(b: FMatrix3): FMatrix3 {
    val sum = FMatrix3()
    CommonOps_FDF3.add(this, b, sum)
    return sum
}

operator fun FMatrix3.minus(b: FMatrix3): FMatrix3 {
    val sum = FMatrix3()
    CommonOps_FDF3.subtract(this, b, sum)
    return sum
}

operator fun FMatrix3.times(t: Float): FMatrix3 {
    return FMatrix3(a1 * t, a2 * t, a3 * t)
}

operator fun FMatrix3.plusAssign(b: FMatrix3) {
    CommonOps_FDF3.addEquals(this, b)
}

operator fun FMatrix3.minusAssign(b: FMatrix3) {
    CommonOps_FDF3.subtractEquals(this, b)
}

operator fun FMatrix3.timesAssign(t: Float) {
    this.a1 *= t
    this.a2 *= t
    this.a3 *= t
}

fun FMatrix3.dot(b: FMatrix3): Float {
    return CommonOps_FDF3.dot(this, b)
}

fun FMatrix3.norm(): Float {
    return sqrt(this.a1 * this.a1 + this.a2 * this.a2 + this.a3 * this.a3)
}

fun FMatrix3.squaredNorm(): Float {
    return this.a1 * this.a1 + this.a2 * this.a2 + this.a3 * this.a3
}

fun FMatrix3.normalize(): FMatrix3 {
    val abs = norm()
    if (abs > 1e-6f) {
        val normed = FMatrix3(this)
        normed *= (1 / abs)
        return normed
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun FMatrix3.normalizeInPlace(): FMatrix3 {
    val norm = this.norm()
    if (norm > 1e-6f) {
        this *= (1 / norm)
        return this
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun FMatrix3.prettyPrint(): String {
    return "Dense 32-bit float Mat 3 x 1 =\n${a1}\n${a2}\n${a3}"
}
