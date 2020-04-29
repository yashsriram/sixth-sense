package extensions

import processing.core.PApplet

fun PApplet.circleXZ(x: Float, z: Float, radius: Float) {
    beginShape()
    val resolution = 20
    for (i in 1..resolution) {
        val theta = 2 * PApplet.PI / (resolution - 1) * i
        vertex(x + radius * PApplet.cos(theta), 0f, z + radius * PApplet.sin(theta))
    }
    endShape(PApplet.CLOSE)
}
