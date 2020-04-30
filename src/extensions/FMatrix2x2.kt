package extensions

import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2
import org.ejml.dense.fixed.CommonOps_FDF2

operator fun FMatrix2x2.times(b: FMatrix2): FMatrix2 {
    val product = FMatrix2()
    CommonOps_FDF2.mult(this, b, product)
    return product
}

fun FMatrix2x2.determinant(): Float {
    return CommonOps_FDF2.det(this)
}

fun FMatrix2x2.inverse(): FMatrix2x2 {
    val inverted = FMatrix2x2()
    val isSuccess = CommonOps_FDF2.invert(this, inverted)
    if (!isSuccess) {
        throw IllegalArgumentException("Can't invert matrix")
    }
    return inverted
}

fun FMatrix2x2.prettyPrint(): String {
    return "Dense 32-bit float Mat 2 x 2 =\n${a11}\t${a12}\n${a21}\t${a22}"

}
