package robot.planning

import extensions.minus
import org.ejml.data.FMatrix2
import processing.core.PApplet
import java.lang.Float.min

class HitGrid(private val applet: PApplet,
              private val minCorner: FMatrix2,
              private val maxCorner: FMatrix2,
              private val numCellsX: Int,
              private val numCellsZ: Int) {

    companion object {
        var DRAW = true
        const val THRESHOLD_COUNT = 50
    }

    private val cellSizeX = (maxCorner.a1 - minCorner.a1) / numCellsX
    private val cellSizeZ = (maxCorner.a2 - minCorner.a2) / numCellsZ
    var maxCount = 0

    // Hits are stored in X-major order
    private val hitMap = arrayListOf<IntArray>()

    init {
        for (i in 0 until numCellsZ) {
            hitMap.add(IntArray(numCellsX))
        }
    }

    fun addHit(point: FMatrix2) {
        // If outside bounds ignore
        if (point.a1 < minCorner.a1 || point.a2 < minCorner.a2 || point.a1 > maxCorner.a1 || point.a2 > maxCorner.a2) {
            return
        }
        val distances = point - minCorner
        val xIndex2D = (distances.a1 / cellSizeX).toInt()
        val zIndex2D = (distances.a2 / cellSizeZ).toInt()
        hitMap[zIndex2D][xIndex2D] = hitMap[zIndex2D][xIndex2D] + 1
        if (hitMap[zIndex2D][xIndex2D] > maxCount) {
            maxCount = hitMap[zIndex2D][xIndex2D]
        }
    }

    fun draw() {
        if (!DRAW) {
            return
        }

        // Boundary
        applet.noFill()
        applet.stroke(0f, 0f, 1f)
        applet.beginShape(PApplet.QUADS)
        applet.vertex(minCorner.a1, 0f, minCorner.a2)
        applet.vertex(minCorner.a1, 0f, maxCorner.a2)
        applet.vertex(maxCorner.a1, 0f, maxCorner.a2)
        applet.vertex(maxCorner.a1, 0f, minCorner.a2)
        applet.endShape()

        // Heat map
        applet.noStroke()
        applet.beginShape(PApplet.QUADS)
        for (i in 0 until numCellsZ) {
            for (j in 0 until numCellsX) {
                val count = hitMap[i][j]
                if (count == 0) {
                    continue
                }
                val r = min(count.toFloat() / THRESHOLD_COUNT, 1f)
                applet.fill(r, 0f, 0f)
                val topLeftX = minCorner.a1 + j * cellSizeX
                val topLeftZ = minCorner.a2 + i * cellSizeZ
                applet.vertex(topLeftX, 0f, topLeftZ)
                applet.vertex(topLeftX, 0f, topLeftZ + cellSizeZ)
                applet.vertex(topLeftX + cellSizeX, 0f, topLeftZ + cellSizeZ)
                applet.vertex(topLeftX + cellSizeX, 0f, topLeftZ)
            }
        }
        applet.endShape()
    }
}