package scratch

import extensions.prettyPrint
import org.ejml.data.FMatrix2

data class Foo(val a: Int, val b: Int)

fun main() {
    var A = Foo(1, 2)
    println("A : $A")
    val B = A
    println("B : $B")
    A = Foo(2, 3)
    println("A : $A")
    println("B : $B")

    println("${(5.7 / 2).toInt()}")
    println("${17 / 4}")

    val list = arrayListOf<FMatrix2>()
    val a = FMatrix2(1f, 1f)
    list.add(a)
    a.set(100f, 101f)
    println(a.prettyPrint())
    println(list[0].prettyPrint())

    val l = mutableListOf(0, 1, 2, 3, 4)
    println(l)
    println(l.removeAt(1))
    println(l)
}