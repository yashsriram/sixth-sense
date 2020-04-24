package simulator

import org.ejml.data.DMatrix2

abstract class Obstacle {
    abstract fun shortestRayDistanceFrom(position: DMatrix2, orientation: DMatrix2): Double
    abstract fun shortestDistanceFrom(position: DMatrix2): Double
    abstract fun draw()
}