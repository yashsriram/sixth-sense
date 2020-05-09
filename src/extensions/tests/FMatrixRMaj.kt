package extensions.tests

import extensions.*
import org.ejml.data.FMatrixRMaj
import org.ejml.dense.row.CommonOps_FDRM

fun main() {
    val A = FMatrixRMaj(
            arrayOf(
                    floatArrayOf(1f, 2f, 3f, 5f),
                    floatArrayOf(3f, 4f, 10f, 6f),
                    floatArrayOf(5f, 6f, 7f, 8f),
                    floatArrayOf(7f, 8f, 9f, 10f)
            )
    )
    println("A = ${A}")
    println("A 1 col mean = ${A.columnWiseMean(1)}")
    println("A transpose : ${A.transpose()}")
    println("A det: ${A.determinant()}")
    println("A inv : ${A.inverse()}")
    println("A inv det : ${A.inverse().determinant()}")
    println("2x3 block @ 1,1 of A: ${A[1, 1, 2, 3]}")
    A[1, 1, 2, 3] = A[0, 0, 2, 3]
    println("After setting 2x3 block @ 0,0 to 2x3 block @ 1,1 of A, we have A as ${A}")
    val C = A
    C[1, 1] = -9f
    println("A changed by reference: ${A} ${C}")
    println("A == C: structural eq ${A == C}")
    println("A === C: referential eq ${A === C}")
    val I = CommonOps_FDRM.identity(4)
    val B = FMatrixRMaj(
            arrayOf(
                    floatArrayOf(1f, 2f, 1f, 2f),
                    floatArrayOf(1f, 2f, 1f, 2f),
                    floatArrayOf(1f, 2f, 1f, 2f),
                    floatArrayOf(1f, 2f, 1f, 2f)
            )
    )
    println("A: ${A}")
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
    println("A * 2: ${A * 2f}")
    A *= 2f
    println("A *= 2: ${A}")
    println("B : ${B}")
    println("vector 2 norm(B) : ${B.norm()}")
}