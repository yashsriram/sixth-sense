package robot.planning

import extensions.minus
import extensions.norm
import extensions.prettyPrint
import org.ejml.data.FMatrix2
import processing.core.PApplet
import java.lang.Float.min
import java.util.*
import kotlin.math.ceil


class HitGrid(private val minCorner: FMatrix2, private val maxCorner: FMatrix2,
              private val numCellsX: Int, private val numCellsY: Int) {
    private val cellSizeX = (maxCorner.a1 - minCorner.a1) / numCellsX
    private val cellSizeY = (maxCorner.a2 - minCorner.a2) / numCellsY
    val hitsAt = MutableList(numCellsX * numCellsY) { 0 }
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

    fun addHit(point: FMatrix2, agentRadius: Float) {
        if (point.a1 < minCorner.a1 || point.a2 < minCorner.a2 || point.a1 > maxCorner.a1 || point.a2 > maxCorner.a2) {
            return
        }
        val position = point - minCorner
        val x = (position.a1 / cellSizeX).toInt()
        val y = (position.a2 / cellSizeY).toInt()
        val upperBoundRangeX = ceil(agentRadius / cellSizeX).toInt()
        val upperBoundRangeY = ceil(agentRadius / cellSizeY).toInt()
        for (i in x - upperBoundRangeX..x + upperBoundRangeX) {
            for (j in y - upperBoundRangeY..y + upperBoundRangeY) {
                if ((i in 0 until numCellsX) && (j in 0 until numCellsY)) {
                    this[i, j]++
                    if (this[i, j] > maxCount) {
                        maxCount = this[i, j]
                    }
                }
            }
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

    fun coordinatesOf(cells: List<Int>): MutableList<FMatrix2> {
        val path = mutableListOf<FMatrix2>()
        for (cell in cells) {
            path.add(coordinateOf(cell))
        }
        return path
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

    private fun search(fringe: Queue<Int>, start: Int, goal: Int): MutableList<Int> {
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
                return searchStateAt[current]!!.pathFromStart
            }
            // Update fringe
            for (neighbour in neighbours(current)) {
                if (hitsAt[neighbour] == 0 && !searchStateAt.containsKey(neighbour)) {
                    addToFringe(fringe, current, neighbour, goal)
                }
            }
        }
        println("Could not reach finish, # vertices explored: $numVerticesExplored")
        return Collections.singletonList(start)
    }

    fun indexOf(position: FMatrix2): Int {
        if (position.a1 < minCorner.a1
                || position.a2 < minCorner.a2
                || position.a1 > maxCorner.a1
                || position.a2 > maxCorner.a2) {
            throw IllegalAccessException("Out of range position")
        }
        val relativePosition = position - minCorner
        val xStart = (relativePosition.a1 / cellSizeX).toInt()
        val yStart = (relativePosition.a2 / cellSizeY).toInt()
        return xStart + yStart * numCellsX
    }

    fun aStar(start: FMatrix2, goal: FMatrix2): MutableList<Int> {
        println("A*")
        println("Resetting search states of vertices")
        searchStateAt.clear()
        return search(PriorityQueue<Int>(kotlin.Comparator { v1, v2 ->
            ((searchStateAt[v1]!!.distanceFromStart + searchStateAt[v1]!!.heuristicDistanceToFinish)
                    - (searchStateAt[v2]!!.distanceFromStart + searchStateAt[v2]!!.heuristicDistanceToFinish)).toInt()
        }), indexOf(start), indexOf(goal))
    }

    fun areHitsOn(end1Index: Int, end2Index: Int): Boolean {
        val x1 = end1Index % numCellsX
        val y1 = end1Index / numCellsX
        val x2 = end2Index % numCellsX
        val y2 = end2Index / numCellsX

        val dy = PApplet.abs(y2 - y1)
        val dx = PApplet.abs(x2 - x1)
        val dyPower2 = dy shl 1
        val dxPower2 = dx shl 1
        val xIncrement = if (x1 < x2) 1 else -1
        val yIncrement = if (y1 < y2) 1 else -1

        var d = 0
        var cursorX = x1
        var cursorY = y1

        if (dy <= dx) {
            while (true) {
                if (hitsAt[cursorX + cursorY * numCellsX] > 0) {
                    return true
                }
                if (cursorX == x2) break
                cursorX += xIncrement
                d += dyPower2
                if (d > dx) {
                    cursorY += yIncrement
                    d -= dxPower2
                }
            }
        } else {
            while (true) {
                if (hitsAt[cursorX + cursorY * numCellsX] > 0) {
                    return true
                }
                if (cursorY == y2) break
                cursorY += yIncrement
                d += dxPower2
                if (d > dy) {
                    cursorX += xIncrement
                    d -= dyPower2
                }
            }
        }
        return false
    }

    fun prettyPrint() {
        for (i in 0 until numCellsX) {
            for (j in 0 until numCellsY) {
                if (hitsAt[i + j * numCellsX] > 0) {
                    print("x")
                } else {
                    print("_")
                }
            }
            println()
        }
    }

    companion object {
        var DRAW = true
        var THRESHOLD_COUNT = 50
    }
}

fun main() {
    val hitGrid = HitGrid(FMatrix2(-10f, -10f), FMatrix2(10f, 10f), 10, 10)
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
    val start = FMatrix2(-10f, -10f)
    val goal = FMatrix2(9.9f, 9.9f)
    println(hitGrid.indexOf(start))
    println(hitGrid.indexOf(goal))
    val cells = hitGrid.aStar(start, goal)
    val path = hitGrid.coordinatesOf(cells)
    for (milestone in path) {
        println(milestone.prettyPrint())
    }
    hitGrid.addHit(FMatrix2(0f, 0f), 1f)
    hitGrid.prettyPrint()
    println(hitGrid.areHitsOn(0, 1))
    println(hitGrid.areHitsOn(0, 11))
    println(hitGrid.areHitsOn(0, 22))
    println(hitGrid.areHitsOn(0, 33))
    println(hitGrid.areHitsOn(0, 44))
    println(hitGrid.areHitsOn(41, 44))
    println(hitGrid.areHitsOn(49, 47))
    println(hitGrid.areHitsOn(48, 46))
}
