package tests

import extensions.*
import org.ejml.data.DMatrix3

fun main() {
    val E = DMatrix3(1.0, 5.0, 11.0)
    val F = DMatrix3(10.0, -1.0, 7.0)
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
    println("E.normalize() = ${E.normalize().prettyPrint()}")
    E.normalizeInPlace()
    println("E = ${E.prettyPrint()}")
    println("||E|| = ${E.norm()}")
}