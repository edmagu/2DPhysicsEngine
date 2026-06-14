package engine;

public final class CollisionInfo {

    // Whether the two bodies are currently overlapping.
    public final boolean colliding;

    // Direction used to separate body A from body B.
    public final Vector2 normal;

    // How deeply the objects overlap.
    public final double penetration;

    // Approximate point where the collision happened.
    public final Vector2 contactPoint;

    /**
     * Shared empty collision result.
     *
     * Instead of creating a new "not colliding" object every time, the engine reuses
     * this one. That avoids unnecessary object creation during collision checks.
     */
    private static final CollisionInfo NONE =
        new CollisionInfo(false, new Vector2(0, 0), 0, new Vector2(0, 0));

    /**
     * Stores the result of a collision test.
     *
     * The physics world needs more than just true/false. It needs the normal,
     * penetration depth, and contact point so it can push objects apart and apply
     * collision impulses.
     */
    public CollisionInfo(boolean colliding, Vector2 normal, double penetration, Vector2 contactPoint) {
        this.colliding = colliding;
        this.normal = normal;
        this.penetration = penetration;
        this.contactPoint = contactPoint;
    }

    public static CollisionInfo none() {
        return NONE;
    }
}