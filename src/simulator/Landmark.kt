package simulator

import math.Vec2

abstract class Landmark {
    abstract fun shortestRayDistanceFrom(position: Vec2, orientation: Vec2): Double
    abstract fun shortestDistanceFrom(position: Vec2): Double
    abstract fun draw()
}