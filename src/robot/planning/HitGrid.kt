package robot.planning

import extensions.minus
import extensions.norm
import extensions.prettyPrint
import org.ejml.data.FMatrix2
import processing.core.PApplet
import java.lang.Float.min
import java.util.*


class HitGrid(private val minCorner: FMatrix2, private val maxCorner: FMatrix2,
              private val numCellsX: Int, private val numCellsY: Int) {
    private val cellSizeX = (maxCorner.a1 - minCorner.a1) / numCellsX
    private val cellSizeY = (maxCorner.a2 - minCorner.a2) / numCellsY
    private val hitsAt = MutableList(numCellsX * numCellsY) { 0 }
    private var maxCount = 0

    private data class SearchState(val distanceFromStart: Float,
                                   val heuristicDistanceToFinish: Float,
                                   val pathFromStart: MutableList<Int>)

    private val searchStateAt = hashMapOf<Int, SearchState>()

    operator fun get(x: Int, y: Int): Int {
        return hitsAt[y * numCellsX + x]
    }

    operator fun set(x: Int, y: Int, value: Int) {
        hitsAt[y * numCellsX + x] = value
    }

    fun addHit(point: FMatrix2) {
        if (point.a1 < minCorner.a1 || point.a2 < minCorner.a2 || point.a1 > maxCorner.a1 || point.a2 > maxCorner.a2) {
            return
        }
        val position = point - minCorner
        val x = (position.a1 / cellSizeX).toInt()
        val y = (position.a2 / cellSizeY).toInt()
        this[x, y]++
        if (this[x, y] > maxCount) {
            maxCount = this[x, y]
        }
    }

    fun draw(applet: PApplet) {
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

    fun neighbours(index: Int): List<Int> {
        val x = index % numCellsX
        val y = index / numCellsX
        val answer = mutableListOf<Int>()
        if (x + 1 < numCellsX) {
            answer.add((x + 1) + numCellsX * (y))
            if (y + 1 < numCellsY) {
                answer.add((x + 1) + numCellsX * (y + 1))
            }
            if (y > 0) {
                answer.add((x + 1) + numCellsX * (y - 1))
            }
        }
        if (x > 0) {
            answer.add((x - 1) + numCellsX * (y))
            if (y + 1 < numCellsY) {
                answer.add((x - 1) + numCellsX * (y + 1))
            }
            if (y > 0) {
                answer.add((x - 1) + numCellsX * (y - 1))
            }
        }
        if (y + 1 < numCellsY) {
            answer.add((x) + numCellsX * (y + 1))
        }
        if (y > 0) {
            answer.add((x) + numCellsX * (y - 1))
        }
        return answer
    }

    fun dist(i: Int, j: Int): Float {
        val pointI = FMatrix2((i % numCellsX).toFloat() * cellSizeX, (i / numCellsX).toFloat() * cellSizeY)
        val pointJ = FMatrix2((j % numCellsX).toFloat() * cellSizeX, (j / numCellsX).toFloat() * cellSizeY)
        return (pointI - pointJ).norm()
    }

    fun coordinateOf(index: Int): FMatrix2 {
        val i = index % numCellsX
        val j = index / numCellsX
        val x = minCorner.a1 + (i + 0.5f) * cellSizeX
        val y = minCorner.a2 + (j + 0.5f) * cellSizeY
        return FMatrix2(x, y)
    }

    private fun addToFringe(fringe: Queue<Int>, current: Int, neighbour: Int, goal: Int) {
        val currentSearchState = searchStateAt[current]
        val pathFromStartToNeighbour = currentSearchState!!.pathFromStart.toMutableList()
        pathFromStartToNeighbour.add(neighbour)
        searchStateAt[neighbour] = SearchState(
                currentSearchState.distanceFromStart + dist(current, neighbour),
                dist(neighbour, goal),
                pathFromStartToNeighbour
        )
        fringe.add(neighbour)
    }

    private fun search(fringe: Queue<Int>, start: Int, goal: Int): List<FMatrix2> {
        var numVerticesExplored = 0
        // Add start to fringe
        searchStateAt[start] = SearchState(0f, dist(start, goal), mutableListOf())
        addToFringe(fringe, start, start, goal)
        while (fringe.size > 0) { // Pop one vertex
            val current = fringe.remove()
            numVerticesExplored++
            // Check if finish
            if (current == goal) {
                println("Reached finish, # vertices explored: $numVerticesExplored")
                val path = mutableListOf<FMatrix2>()
                for (milestone in searchStateAt[current]!!.pathFromStart) {
                    path.add(coordinateOf(milestone))
                }
                return path
            }
            // Update fringe
            for (neighbour in neighbours(current)) {
                if (hitsAt[neighbour] == 0 && !searchStateAt.containsKey(neighbour)) {
                    addToFringe(fringe, current, neighbour, goal)
                }
            }
        }
        println("Could not reach finish, # vertices explored: $numVerticesExplored")
        return Collections.singletonList(coordinateOf(start))
    }

    fun aStar(xStart: Int, yStart: Int, xGoal: Int, yGoal: Int): List<FMatrix2> {
        println("BFS")
        println("Resetting search states of vertices")
        searchStateAt.clear()
        return search(PriorityQueue<Int>(kotlin.Comparator { v1, v2 ->
            ((searchStateAt[v1]!!.distanceFromStart + searchStateAt[v1]!!.heuristicDistanceToFinish)
                    - (searchStateAt[v2]!!.distanceFromStart + searchStateAt[v2]!!.heuristicDistanceToFinish)).toInt()
        }), xStart + yStart * numCellsX, xGoal + yGoal * numCellsX)
    }

    companion object {
        var DRAW = true
        const val THRESHOLD_COUNT = 50
    }

}

fun main() {
    val hitGrid = HitGrid(FMatrix2(-10f, -5f), FMatrix2(10f, 5f), 10, 10)
    println(hitGrid.neighbours(0))
    println(hitGrid.neighbours(1))
    println(hitGrid.neighbours(10))
    println(hitGrid.neighbours(11))
    println(hitGrid.neighbours(9))
    println(hitGrid.neighbours(8))
    println(hitGrid.neighbours(19))
    println(hitGrid.neighbours(18))
    println(hitGrid.neighbours(99))
    println(hitGrid.neighbours(90))
    println(hitGrid.dist(0, 1))
    println(hitGrid.dist(0, 10))
    println(hitGrid.dist(0, 11))
    println(hitGrid.coordinateOf(99).prettyPrint())
    val path = hitGrid.aStar(0, 0, 99, 0)
    for (milestone in path) {
        println(milestone.prettyPrint())
    }
}
