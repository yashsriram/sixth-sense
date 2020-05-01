package extensions.tests

import extensions.*
import org.ejml.data.FMatrix2
import org.ejml.data.FMatrix2x2

fun main() {
    val A = FMatrix2x2(
            10f, 2f,
            -1f, 1f
    )
    println("A = ${A.prettyPrint()}")
    val X = FMatrix2(1f, 2f)
    println("X = ${X.prettyPrint()}")
    val B = A * X
    println("A * X = ${B.prettyPrint()}")

    val E = FMatrix2(1f, 5f)
    val F = FMatrix2(10f, -1f)
    println("E = ${E.prettyPrint()}")
    println("F = ${F.prettyPrint()}")
    println("E + F = ${(E + F).prettyPrint()}")
    println("E - F = ${(E - F).prettyPrint()}")
    E += F
    println("E += F = ${E.prettyPrint()}")
    E -= F
    println("E -= F = ${E.prettyPrint()}")
    println("E * 2 = ${(E * 2f).prettyPrint()}")
    E *= 2f
    println("E *= 2 = ${E.prettyPrint()}")
    println("||E|| = ${E.norm()}")
    println("E.normalize() = ${E.normalize().prettyPrint()}")
    E.normalizeInPlace()
    println("E = ${E.prettyPrint()}")
    println("||E|| = ${E.norm()}")
}