package com.edmagu.twodphysics.engine;

public class RopeJoint {

    // Bodies connected by the rope.
    private PhysicsBody a;
    private PhysicsBody b;

    // Anchor points stored in each body's local space.
    private Vector2 localAnchorA;
    private Vector2 localAnchorB;

    // Rope length when it was created.
    private double restLength;

    // Spring strength.
    private double stiffness;

    // Reduces rope oscillation.
    private double damping;

    /**
     * Creates a rope between two bodies.
     *
     * The anchors are passed in as world points, but they are immediately converted
     * into local body space. That way, the anchors stay attached to the same part of
     * each body even if the bodies move or rotate.
     */
    public RopeJoint(
            PhysicsBody a,
            PhysicsBody b,
            Vector2 worldAnchorA,
            Vector2 worldAnchorB,
            double stiffness,
            double damping) {
        this.a = a;
        this.b = b;

        this.localAnchorA = worldToLocal(a, worldAnchorA);
        this.localAnchorB = worldToLocal(b, worldAnchorB);

        // The starting distance becomes the rope's natural/resting length.
        this.restLength = worldAnchorB.subtract(worldAnchorA).magnitude();

        this.stiffness = stiffness;
        this.damping = damping;
    }

    /**
     * Applies rope forces for one physics step.
     *
     * This behaves more like a spring-damper than a perfectly rigid rope.
     * If the rope is stretched past its rest length, it pulls the bodies together.
     * The damping term reduces bouncing by resisting motion along the rope.
     */
    public void solve(double dt) {
        Vector2 anchorA = getWorldAnchorA();
        Vector2 anchorB = getWorldAnchorB();

        Vector2 delta = anchorB.subtract(anchorA);
        double distance = delta.magnitude();

        if (distance < 1e-8) {
            return;
        }

        // Unit direction from anchor A to anchor B.
        Vector2 direction = delta.multiply(1.0 / distance);

        // Positive stretch means the rope is longer than its resting length.
        double stretch = distance - restLength;

        /*
         * Relative velocity tells us whether the bodies are moving toward or away
         * from each other along the rope direction.
         */
        Vector2 relativeVelocity = b.getLinearVelocity().subtract(a.getLinearVelocity());
        double velocityAlongRope = relativeVelocity.dot(direction);

        /*
         * Spring force:
         *
         * stretch * stiffness pulls harder the more the rope is stretched.
         *
         * Damping force:
         *
         * velocityAlongRope * damping reduces oscillation and helps stop the rope
         * from bouncing forever.
         */
        double forceAmount = stretch * stiffness + velocityAlongRope * damping;
        Vector2 force = direction.multiply(forceAmount);

        
        /*
         * Apply equal and opposite forces.
         *
         * This follows Newton's Third Law: if A pulls B, B pulls A back.
         */
        a.applyForce(force);
        b.applyForce(force.negate());
    }

    /**
     * Converts a world point into a body's local coordinate space.
     *
     * This removes the body's position and rotation from the point.
     */
    private Vector2 worldToLocal(PhysicsBody body, Vector2 worldPoint) {
        Vector2 p = worldPoint.subtract(body.getPosition());

        double cos = Math.cos(-body.getRotation());
        double sin = Math.sin(-body.getRotation());

        return new Vector2(
                p.x * cos - p.y * sin,
                p.x * sin + p.y * cos);
    }

    /**
     * Converts a local anchor point back into world space.
     *
     * This applies the body's current rotation and position.
     */
    private Vector2 localToWorld(PhysicsBody body, Vector2 localPoint) {
        double cos = Math.cos(body.getRotation());
        double sin = Math.sin(body.getRotation());

        return body.getPosition().add(new Vector2(
                localPoint.x * cos - localPoint.y * sin,
                localPoint.x * sin + localPoint.y * cos));
    }

    public PhysicsBody getA() {
        return a;
    }

    public PhysicsBody getB() {
        return b;
    }

    public Vector2 getWorldAnchorA() {
        return localToWorld(a, localAnchorA);
    }

    public Vector2 getWorldAnchorB() {
        return localToWorld(b, localAnchorB);
    }
}