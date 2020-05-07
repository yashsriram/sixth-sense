package extensions

import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM
import kotlin.math.sqrt

operator fun FMatrixRMaj.get(i: Int, j: Int, numRows: Int, numCols: Int): FMatrixRMaj {
    val block = FMatrixRMaj(numRows, numCols)
    CommonOps_FDRM.extract(this, i, j, block)
    return block
}

operator fun FMatrixRMaj.set(i: Int, j: Int, numRows: Int, numCols: Int, rhs: FMatrixRMaj) {
    if (rhs.numRows != numRows || rhs.numCols != numCols) {
        throw IllegalArgumentException("Incompatible block assignment")
    }
    CommonOps_FDRM.insert(rhs, this, i, j)
}

operator fun FMatrixRMaj.plus(b: FMatrixRMaj): FMatrixRMaj {
    val sum = FMatrixRMaj(this.numRows, this.numCols)
    CommonOps_FDRM.add(this, b, sum)
    return sum
}

operator fun FMatrixRMaj.minus(b: FMatrixRMaj): FMatrixRMaj {
    val difference = FMatrixRMaj(this.numRows, this.numCols)
    CommonOps_FDRM.subtract(this, b, difference)
    return difference
}

operator fun FMatrixRMaj.times(b: FMatrixRMaj): FMatrixRMaj {
    val product = FMatrixRMaj(this.numRows, b.numCols)
    CommonOps_FDRM.mult(this, b, product)
    return product
}

operator fun FMatrixRMaj.times(t: Float): FMatrixRMaj {
    val scaled = this.createLike()
    CommonOps_FDRM.scale(t, this, scaled)
    return scaled
}

operator fun FMatrixRMaj.plusAssign(b: FMatrixRMaj) {
    CommonOps_FDRM.addEquals(this, b)
}

fun FMatrixRMaj.norm(): Float {
    val size = this.numElements
    var sumOfSquares = 0.0
    for (i in 0 until size) {
        sumOfSquares += (this.data[i] * this.data[i])
    }
    return sqrt(sumOfSquares).toFloat()
}

fun FMatrixRMaj.transpose(): FMatrixRMaj {
    val transpose = FMatrixRMaj(this.numCols, this.numRows)
    CommonOps_FDRM.transpose(this, transpose)
    return transpose
}

fun FMatrixRMaj.inverse(): FMatrixRMaj {
    val inverted = this.createLike()
    val isSuccess = CommonOps_FDRM.invert(this, inverted)
    if (!isSuccess) {
        throw IllegalArgumentException("Can't invert matrix")
    }
    return inverted
}

fun FMatrixRMaj.determinant(): Float {
    return CommonOps_FDRM.det(this)
}

fun FMatrixRMaj.columnWiseMean(col: Int): Float {
    if (col >= this.numCols) {
        throw IllegalArgumentException("Bad col number")
    }
    var sum = 0f
    for (i in 0 until this.numRows) {
        sum += get(i, col)
    }
    return sum / numRows
}
