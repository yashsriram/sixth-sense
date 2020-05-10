package robot.planning

import extensions.minus
import org.ejml.data.FMatrix2
import processing.core.PApplet
import java.lang.Float.min


class HitGrid(private val applet: PApplet,
              private val minCorner: FMatrix2, private val maxCorner: FMatrix2,
              private val numCellsX: Int, private val numCellsY: Int) {
    // Cell size
    private val cellSizeX = (maxCorner.a1 - minCorner.a1) / numCellsX
    private val cellSizeY = (maxCorner.a2 - minCorner.a2) / numCellsY
    // Stored in X-major order
    private val hitsAt = MutableList(numCellsX * numCellsY) { 0 }

    var maxCount = 0

    companion object {
        var DRAW = true
        const val THRESHOLD_COUNT = 50
    }

    operator fun get(x: Int, y: Int): Int {
        return hitsAt[y * numCellsX + x]
    }

    operator fun set(x: Int, y: Int, value: Int) {
        hitsAt[y * numCellsX + x] = value
    }

    fun addHit(point: FMatrix2) {
        // If outside bounds ignore
        if (point.a1 < minCorner.a1 || point.a2 < minCorner.a2 || point.a1 > maxCorner.a1 || point.a2 > maxCorner.a2) {
            return
        }
        val distances = point - minCorner
        val xIndex = (distances.a1 / cellSizeX).toInt()
        val yIndex = (distances.a2 / cellSizeY).toInt()
        this[xIndex, yIndex] = this[xIndex, yIndex] + 1
        if (this[xIndex, yIndex] > maxCount) {
            maxCount = this[xIndex, yIndex]
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
        for (i in 0 until numCellsX) {
            for (j in 0 until numCellsY) {
                val count = this[i, j]
                if (count == 0) {
                    continue
                }
                val r = min(count.toFloat() / THRESHOLD_COUNT, 1f)
                applet.fill(r, 0f, 0f)
                val topLeftX = minCorner.a1 + i * cellSizeX
                val topLeftZ = minCorner.a2 + j * cellSizeY
                applet.vertex(topLeftX, 0f, topLeftZ)
                applet.vertex(topLeftX, 0f, topLeftZ + cellSizeY)
                applet.vertex(topLeftX + cellSizeX, 0f, topLeftZ + cellSizeY)
                applet.vertex(topLeftX + cellSizeX, 0f, topLeftZ)
            }
        }
        applet.endShape()
    }

//    private fun addToFringe(fringe: Queue<Vertex>, current: Vertex, next: Vertex) {
//        next.searchState.distanceFromStart = current.searchState.distanceFromStart + next.position.minus(current.position).norm()
//        fringe.add(next)
//        next.searchState.addToFringeFrom(current)
//    }
//
//    private fun search(fringe: Queue<Vertex>, agentIndex: Int): List<Vec3?>? {
//        var numVerticesExplored = 0
//        // Add start to fringe
//        addToFringe(fringe, starts.get(agentIndex), starts.get(agentIndex))
//        while (fringe.size > 0) { // Pop one vertex
//            val current: Vertex = fringe.remove()
//            numVerticesExplored++
//            // Check if finish
//            if (current.isFinishVertex()) {
//                PApplet.println("Reached finish, # vertices explored: $numVerticesExplored")
//                return finishes.get(agentIndex).searchState.pathFromStart
//            }
//            // Mark this vertex as explored
//            current.searchState.setExplored()
//            // Update fringe
//            for (neighbour in current.neighbours) {
//                if (neighbour.isOutsideObstacle && !neighbour.searchState.isExplored) {
//                    addToFringe(fringe, current, neighbour)
//                }
//            }
//        }
//        PApplet.println("Could not reach finish, # vertices explored: $numVerticesExplored")
//        return listOf<Vec3>(starts.get(agentIndex).position)
//    }
//
//    fun bfs(agentIndex: Int): List<Vec3?>? {
//        PApplet.println("BFS")
//        resetSearchState(finishes.get(agentIndex).position)
//        return search(LinkedList<Vertex>(), agentIndex)
//    }

}