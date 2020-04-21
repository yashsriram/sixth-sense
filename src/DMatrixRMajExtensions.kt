import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.CommonOps_DDRM
import java.lang.IllegalArgumentException

operator fun DMatrixRMaj.get(i: Int, j: Int, numRows: Int, numCols: Int): DMatrixRMaj {
    val block = DMatrixRMaj(numRows, numCols)
    CommonOps_DDRM.extract(this, i, j, block)
    return block
}

operator fun DMatrixRMaj.set(i: Int, j: Int, numRows: Int, numCols: Int, rhs: DMatrixRMaj) {
    if (rhs.numRows != numRows || rhs.numCols != numCols) {
        throw IllegalArgumentException("Incompatible block assignment")
    }
    CommonOps_DDRM.insert(rhs, this, i, j)
}

operator fun DMatrixRMaj.plus(b: DMatrixRMaj): DMatrixRMaj {
    val sum = DMatrixRMaj(this.numRows, this.numCols)
    CommonOps_DDRM.add(this, b, sum)
    return sum
}

operator fun DMatrixRMaj.minus(b: DMatrixRMaj): DMatrixRMaj {
    val difference = DMatrixRMaj(this.numRows, this.numCols)
    CommonOps_DDRM.subtract(this, b, difference)
    return difference
}

operator fun DMatrixRMaj.plusAssign(b: DMatrixRMaj) {
    CommonOps_DDRM.addEquals(this, b)
}

fun DMatrixRMaj.transpose(): DMatrixRMaj {
    val transpose = this.createLike()
    CommonOps_DDRM.transpose(this, transpose)
    return transpose
}

fun DMatrixRMaj.inverse(): DMatrixRMaj {
    val inverted = this.createLike()
    val isSuccess = CommonOps_DDRM.invert(this, inverted)
    if (!isSuccess) {
        throw IllegalArgumentException("Can't invert matrix")
    }
    return inverted
}

fun DMatrixRMaj.determinant(): Double {
    return CommonOps_DDRM.det(this)
}
