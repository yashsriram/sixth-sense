package simulator

import org.ejml.data.FMatrix2

abstract class Obstacle {
    abstract fun shortestRayDistanceFrom(position: FMatrix2, orientation: FMatrix2): Float
    abstract fun shortestDistanceFrom(position: FMatrix2): Float
    abstract fun draw()
}