package scratch

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
}