package simulator.environment;

import math.Vec2;

public abstract class Landmark {
    public abstract double shortestRayDistanceFrom(final Vec2 position, final Vec2 orientation);

    public abstract double shortestDistanceFrom(Vec2 position);

    public abstract void draw();
}
