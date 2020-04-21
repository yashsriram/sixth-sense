package extensions

import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix2x2
import org.ejml.dense.fixed.CommonOps_DDF2

operator fun DMatrix2x2.times(b: DMatrix2): DMatrix2 {
    val product = DMatrix2()
    CommonOps_DDF2.mult(this, b, product)
    return product
}

fun DMatrix2x2.prettyPrint(): String {
    return "Dense 64-bit float Mat 2 x 2 =\n${a11}\t${a12}\n${a21}\t${a22}"

}
