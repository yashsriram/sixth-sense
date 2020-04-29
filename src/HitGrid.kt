import extensions.minus
import org.ejml.data.DMatrix2
import processing.core.PApplet

class HitGrid(private val applet: PApplet,
              private val minCorner: DMatrix2,
              private val maxCorner: DMatrix2,
              private val numCellsX: Int,
              private val numCellsZ: Int) {
    private val cellSizeX = (maxCorner.a1 - minCorner.a1) / numCellsX
    private val cellSizeZ = (maxCorner.a2 - minCorner.a2) / numCellsZ

    // Hits are stored in X-major order
    private val hitMap = hashMapOf<Int, Int>()

    private fun get1DIndex(xIndex: Int, zIndex: Int): Int {
        return zIndex * numCellsX + xIndex
    }

    fun addHit(point: DMatrix2) {
        // If outside bounds ignore
        if (point.a1 < minCorner.a1 || point.a2 < minCorner.a2 || point.a1 > maxCorner.a1 || point.a2 > maxCorner.a2) {
            return
        }
        val distances = point - minCorner
        val xIndex2D = (distances.a1 / cellSizeX).toInt()
        val zIndex2D = (distances.a2 / cellSizeZ).toInt()
        val index1D = get1DIndex(xIndex2D, zIndex2D)
        if (hitMap.containsKey(index1D)) {
            hitMap[index1D] = hitMap[index1D]!! + 1
        } else {
            hitMap[index1D] = 1
        }
    }

    fun draw() {
        // Boundary
        applet.noFill()
        applet.stroke(0f, 0f, 1f)
        applet.beginShape(PApplet.QUADS)
        applet.vertex(minCorner.a1.toFloat(), 0f, minCorner.a2.toFloat())
        applet.vertex(minCorner.a1.toFloat(), 0f, maxCorner.a2.toFloat())
        applet.vertex(maxCorner.a1.toFloat(), 0f, maxCorner.a2.toFloat())
        applet.vertex(maxCorner.a1.toFloat(), 0f, minCorner.a2.toFloat())
        applet.endShape()

        // Heat map
        applet.noStroke()
        applet.beginShape(PApplet.QUADS)
        for (key in hitMap.keys) {
            val i = key % numCellsX
            val j = key / numCellsX
            applet.fill(1f, 0f, 0f)
            val topLeftX = minCorner.a1 + i * cellSizeX
            val topLeftZ = minCorner.a2 + j * cellSizeZ
            applet.vertex(topLeftX.toFloat(), 0f, topLeftZ.toFloat())
            applet.vertex(topLeftX.toFloat(), 0f, topLeftZ.toFloat() + cellSizeZ.toFloat())
            applet.vertex(topLeftX.toFloat() + cellSizeX.toFloat(), 0f, topLeftZ.toFloat() + cellSizeZ.toFloat())
            applet.vertex(topLeftX.toFloat() + cellSizeX.toFloat(), 0f, topLeftZ.toFloat())
        }
        applet.endShape()
    }
}