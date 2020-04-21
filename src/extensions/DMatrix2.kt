package extensions

import org.ejml.data.DMatrix2
import org.ejml.dense.fixed.CommonOps_DDF2
import kotlin.math.sqrt

operator fun DMatrix2.plus(b: DMatrix2): DMatrix2 {
    val sum = DMatrix2()
    CommonOps_DDF2.add(this, b, sum)
    return sum
}

operator fun DMatrix2.minus(b: DMatrix2): DMatrix2 {
    val sum = DMatrix2()
    CommonOps_DDF2.subtract(this, b, sum)
    return sum
}

operator fun DMatrix2.times(t: Double): DMatrix2 {
    return DMatrix2(a1 * t, a2 * t)
}

operator fun DMatrix2.plusAssign(b: DMatrix2) {
    CommonOps_DDF2.addEquals(this, b)
}

operator fun DMatrix2.minusAssign(b: DMatrix2) {
    CommonOps_DDF2.subtractEquals(this, b)
}

operator fun DMatrix2.timesAssign(t: Double) {
    this.a1 *= t
    this.a2 *= t
}

fun DMatrix2.norm(): Double {
    return sqrt(this.a1 * this.a1 + this.a2 * this.a2)
}

fun DMatrix2.normalize(): DMatrix2 {
    val abs = norm()
    if (abs > 1e-6f) {
        val normed = DMatrix2(this)
        normed *= (1 / abs)
        return normed
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun DMatrix2.normalizeInPlace(): DMatrix2 {
    val norm = this.norm()
    if (norm > 1e-6f) {
        this *= (1 / norm)
        return this
    }
    throw IllegalStateException("Attempt to normalize zero vector")
}

fun DMatrix2.prettyPrint(): String {
    return "Dense 64-bit float Mat 2 x 1 =\n${a1}\n${a2}"
}
