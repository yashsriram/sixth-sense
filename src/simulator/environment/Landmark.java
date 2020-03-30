package simulator.environment;

import math.Vec2;

public abstract class Landmark {
    public abstract double shortestRayDistance(final Vec2 eye, final Vec2 forwardDirection);

    public abstract double shortestDistance(Vec2 eye);

    public abstract void draw();
}
