package tests

import extensions.*
import org.ejml.data.DMatrix2
import org.ejml.data.DMatrix2x2

fun main() {
    val A = DMatrix2x2(
            10.0, 2.0,
            -1.0, 1.0
    )
    println("A = ${A.prettyPrint()}")
    val X = DMatrix2(1.0, 2.0)
    println("X = ${X.prettyPrint()}")
    val B = A * X
    println("A * X = ${B.prettyPrint()}")

    val E = DMatrix2(1.0 , 5.0)
    val F = DMatrix2(10.0 , -1.0)
    println("E = ${E.prettyPrint()}")
    println("F = ${F.prettyPrint()}")
    println("E + F = ${(E + F).prettyPrint()}")
    println("E - F = ${(E - F).prettyPrint()}")
    E += F
    println("E += F = ${E.prettyPrint()}")
    E -= F
    println("E -= F = ${E.prettyPrint()}")
    println("E * 2 = ${(E * 2.0).prettyPrint()}")
    E *= 2.0
    println("E *= 2 = ${E.prettyPrint()}")
    println("||E|| = ${E.norm()}")
    E.normInPlace()
    println("E = ${E.prettyPrint()}")
    println("||E|| = ${E.norm()}")
}