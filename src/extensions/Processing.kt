package extensions

import processing.core.PApplet

fun PApplet.circleXZ(x: Double, z: Double, radius: Double) {
    beginShape()
    val resolution = 20
    for (i in 1..resolution) {
        val theta = 2 * PApplet.PI / (resolution - 1) * i
        vertex((x + radius * PApplet.cos(theta)).toFloat(), 0f, (z + radius * PApplet.sin(theta)).toFloat())
    }
    endShape(PApplet.CLOSE)
}
