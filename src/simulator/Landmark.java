package simulator;

import math.Vec2;

public abstract class Landmark {
    abstract double shortestRayDistanceFrom(final Vec2 position, final Vec2 orientation);

    abstract double shortestDistanceFrom(Vec2 position);

    abstract void draw();
}
