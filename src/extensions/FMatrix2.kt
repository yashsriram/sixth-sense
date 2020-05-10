package extensions

import org.ejml.data.FMatrix2
import org.ejml.dense.fixed.CommonOps_FDF2
import kotlin.math.sqrt

operator fun FMatrix2.plus(b: FMatrix2): FMatrix2 {
    val sum = FMatrix2()
    CommonOps_FDF2.add(this, b, sum)
    return sum
}

operator fun FMatrix2.minus(b: FMatrix2): FMatrix2 {
    val sum = FMatrix2()
    CommonOps_FDF2.subtract(this, b, sum)
    return sum
}

operator fun FMatrix2.times(t: Float): FMatrix2 {
    return FMatrix2(a1 * t, a2 * t)
}

operator fun FMatrix2.plusAssign(b: FMatrix2) {
    CommonOps_FDF2.addEquals(this, b)
}

operator fun FMatrix2.minusAssign(b: FMatrix2) {
    CommonOps_FDF2.subtractEquals(this, b)
}

operator fun FMatrix2.timesAssign(t: Float) {
    this.a1 *= t
    this.a2 *= t
}

fun FMatrix2.dist(b: FMatrix2): Float {
    return sqrt((a1 - b.a1) * (a1 - b.a1) + (a2 - b.a2) * (a2 - b.a2))
}

fun FMatrix2.dot(b: FMatrix2): Float {
    return CommonOps_FDF2.dot(this, b)
}

fun FMatrix2.norm(): Float {
    return sqrt(this.a1 * this.a1 + this.a2 * this.a2)
}

fun FMatrix2.squaredNorm(): Float {
    return this.a1 * this.a1 + this.a2 * this.a2
}

fun FMatrix2.normalize(): FMatrix2 {
    val abs = norm()
    if (abs > 1e-6f) {
        val normed = FMatrix2(this)
        normed *= (1 / abs)
        return normed
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun FMatrix2.normalizeInPlace(): FMatrix2 {
    val norm = this.norm()
    if (norm > 1e-6f) {
        this *= (1 / norm)
        return this
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun FMatrix2.prettyPrint(): String {
    return "FMat2={${a1}, ${a2}}"
}
