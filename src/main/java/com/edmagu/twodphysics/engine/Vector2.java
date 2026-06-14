package com.edmagu.twodphysics.engine;

public final class Vector2 {

    // X and Y components are final, so Vector2 objects are immutable.
    public final double x;
    public final double y;

    /**
     * Creates a 2D vector.
     *
     * Vectors are used everywhere in the engine for positions, velocities, forces,
     * normals, contact points, and offsets.
     */
    public Vector2(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Adds two vectors.
     *
     * Used for things like moving a position by a velocity or combining forces.
     */
    public Vector2 add(Vector2 other) {
        return new Vector2(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtracts another vector from this one.
     *
     * This is commonly used to get a direction or offset between two points.
     */
    public Vector2 subtract(Vector2 other) {
        return new Vector2(this.x - other.x, this.y - other.y);
    }

    /**
     * Scales the vector by a number.
     *
     * Used for equations like velocity * dt, force * inverseMass, or normal * impulse.
     */
    public Vector2 multiply(double scalar) {
        return new Vector2(this.x * scalar, this.y * scalar);
    }

    /**
     * Dot product.
     *
     * This measures how much two vectors point in the same direction. In the physics
     * engine, it is used for projections, velocity along a normal, and friction.
     */
    public double dot(Vector2 other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * 2D cross product.
     *
     * In 2D, this returns a scalar instead of a vector. The result represents the
     * rotational effect between two vectors, which is why it is useful for torque
     * and angular impulse calculations.
     */
    public double cross(Vector2 other) {
        return this.x * other.y - this.y * other.x;
    }

    /**
     * Returns the length of the vector.
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Returns the squared length of the vector.
     *
     * This avoids a square root, so it is faster when only comparing distances.
     */
    public double magnitudeSquared() {
        return x * x + y * y;
    }

    /**
     * Returns a unit vector pointing in the same direction.
     *
     * If the vector is almost zero, it returns zero instead of dividing by a tiny
     * number. That avoids unstable math and NaN values.
     */
    public Vector2 normalized() {
        double mag = magnitude();

        if (mag < 1e-10) {
            return new Vector2(0, 0);
        }

        return new Vector2(x / mag, y / mag);
    }

    /**
     * Returns this vector pointing in the opposite direction.
     */
    public Vector2 negate() {
        return new Vector2(-x, -y);
    }

    /**
     * Convenience method for creating a zero vector.
     */
    public static Vector2 zero() {
        return new Vector2(0, 0);
    }
}