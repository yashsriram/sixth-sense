package extensions.tests

import extensions.*
import org.ejml.data.DMatrixRMaj
import org.ejml.dense.row.CommonOps_DDRM

fun main() {
    val A = DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(1.0, 2.0, 3.0, 5.0),
                    doubleArrayOf(3.0, 4.0, 10.0, 6.0),
                    doubleArrayOf(5.0, 6.0, 7.0, 8.0),
                    doubleArrayOf(7.0, 8.0, 9.0, 10.0)
            )
    )
    println("A = ${A}")
    println("A transpose : ${A.transpose()}")
    println("A det: ${A.determinant()}")
    println("A inv : ${A.inverse()}")
    println("2x3 block @ 1,1 of A: ${A[1, 1, 2, 3]}")
    A[1, 1, 2, 3] = A[0, 0, 2, 3]
    println("After setting 2x3 block @ 0,0 to 2x3 block @ 1,1 of A, we have A as ${A}")
    val C = A
    C[1, 1] = -9.0
    println("A changed by reference: ${A} ${C}")
    println("A == C: ${A == C}")
    println("A === C: ${A === C}")
    val I = CommonOps_DDRM.identity(4)
    val B = DMatrixRMaj(
            arrayOf(
                    doubleArrayOf(1.0, 2.0, 1.0, 2.0),
                    doubleArrayOf(1.0, 2.0, 1.0, 2.0),
                    doubleArrayOf(1.0, 2.0, 1.0, 2.0),
                    doubleArrayOf(1.0, 2.0, 1.0, 2.0)
            )
    )
    println("B: ${B}")
    println("I: ${I}")
    println("A - B + I = ${A - B + I}")
    println("A + B - I = ${A + B - I}")
    val D = A + I
    println("D = A + I: ${D}")
    A += I
    println("A += I; A: ${A}")
    for (i in 0.until(A.numRows)) {
        for (j in 0.until(A.numCols)) {
            if (A[i, j] != D[i, j]) {
                println("false")
            }
        }
    }
    println("A == D: ${A == D}: weird!!!")
    println("A === D: ${A === D}")
    println("A : ${A}")
    println("B : ${B}")
    println("A * B: ${A * B}")
    println("A * 2: ${A * 2.0}")
    println("B : ${B}")
    println("vector 2 norm(B) : ${B.norm()}")
}