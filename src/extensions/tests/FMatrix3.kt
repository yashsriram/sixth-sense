package extensions.tests

import extensions.*
import org.ejml.data.FMatrix3

fun main() {
    val E = FMatrix3(1f, 5f, 11f)
    val F = FMatrix3(10f, -1f, 7f)
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