package engine;

import java.util.ArrayList;
import java.util.List;

public class PhysicsWorld {

    // Number of smaller simulation steps inside each update.
    private static final int SUBSTEPS = 16;

    // How strongly overlapping bodies are pushed apart.
    private static final double CORRECTION_PERCENT = 0.7;

    // Small allowed overlap to prevent jitter.
    private static final double CORRECTION_SLOP = 0.001;

    // Minimum impact speed needed before restitution/bounce is used.
    private static final double RESTITUTION_THRESHOLD = 0.0;

    // How many times collision impulses are solved per step.
    private static final int VELOCITY_ITERATIONS = 8;

    // How many times position overlap correction is solved per step.
    private static final int POSITION_ITERATIONS = 4;

    // All bodies currently in the world.
    private final List<PhysicsBody> bodies;

    // All rope constraints currently in the world.
    private final List<RopeJoint> ropes = new ArrayList<>();

    // Gravity applied to dynamic bodies.
    private Vector2 gravity;

    /**
     * Creates an empty physics world with default downward gravity.
     */
    public PhysicsWorld() {
        this.bodies = new ArrayList<>();
        this.gravity = new Vector2(0, -9.81); // m/s
    }

    public void setGravity(Vector2 gravity) {
        this.gravity = gravity;
    }

    public Vector2 getGravity() {
        return gravity;
    }

    public void addBody(PhysicsBody body) {
        bodies.add(body);
    }

    public boolean removeBody(PhysicsBody body) {
        return bodies.remove(body);
    }

    public List<PhysicsBody> getBodies() {
        return bodies;
    }

    /**
     * Adds a rope joint between two bodies.
     *
     * The stiffness and damping are currently hard-coded here, which means all
     * ropes
     * created through this method behave the same way.
     */
    public void addRope(
            PhysicsBody a,
            PhysicsBody b,
            Vector2 anchorA,
            Vector2 anchorB) {
        ropes.add(new RopeJoint(a, b, anchorA, anchorB, 50.0, 500));
    }

    public List<RopeJoint> getRopes() {
        return ropes;
    }

    public void removeRope(RopeJoint rope) {
        ropes.remove(rope);
    }

    public void clearRopes() {
        ropes.clear();
    }

    /**
     * Clears the entire physics world.
     *
     * This removes all bodies and ropes so the engine can rebuild the scene
     * from scratch.
     */
    public void clear() {
        bodies.clear();
        ropes.clear();
    }

    /**
     * Removes all ropes connected to a body.
     *
     * This is important when deleting objects, otherwise ropes could keep
     * references
     * to bodies that no longer exist in the world.
     */
    public void removeRopesConnectedTo(PhysicsBody body) {
        ropes.removeIf(rope -> rope.getA() == body || rope.getB() == body);
    }

    /**
     * Advances the whole physics world by one frame.
     *
     * The timestep is split into substeps to improve stability. Smaller steps make
     * collision detection, collision response, and rope solving behave better,
     * especially when objects are moving quickly.
     *
     * beforeStep is a callback used by the engine to apply extra forces, like the
     * mouse spring, at the right time inside the simulation loop.
     */
    public void update(double dt, Runnable beforeStep) {
        double subDt = dt / SUBSTEPS;

        for (int s = 0; s < SUBSTEPS; s++) {
            if (beforeStep != null) {
                beforeStep.run();
            }

            for (RopeJoint rope : ropes) {
                rope.solve(subDt);
            }

            for (PhysicsBody body : bodies) {
                body.update(subDt, gravity);
            }

            resolveCollisions();
        }
    }

    public boolean isInContact(PhysicsBody a, PhysicsBody b) {
        return CollisionDetector.getCollisionInfo(a, b).colliding;
    }

    /**
     * Resolves all collisions in the world.
     *
     * This is split into two phases:
     *
     * 1. Velocity solving:
     * Applies impulses to make objects bounce, stop moving into each other,
     * and react with friction.
     *
     * 2. Position solving:
     * Pushes objects apart if they are still overlapping.
     *
     * Multiple iterations make the simulation more stable, especially when several
     * objects are touching at the same time.
     */
    private void resolveCollisions() {
        for (int iter = 0; iter < VELOCITY_ITERATIONS; iter++) {
            for (int i = 0; i < bodies.size(); i++) {
                for (int j = i + 1; j < bodies.size(); j++) {
                    PhysicsBody a = bodies.get(i);
                    PhysicsBody b = bodies.get(j);

                    CollisionInfo info = CollisionDetector.getCollisionInfo(a, b);

                    if (info.colliding) {
                        applyCollisionImpulse(a, b, info);
                    }
                }
            }
        }

        for (int iter = 0; iter < POSITION_ITERATIONS; iter++) {
            for (int i = 0; i < bodies.size(); i++) {
                for (int j = i + 1; j < bodies.size(); j++) {
                    PhysicsBody a = bodies.get(i);
                    PhysicsBody b = bodies.get(j);

                    CollisionInfo info = CollisionDetector.getCollisionInfo(a, b);

                    if (info.colliding) {
                        positionalCorrection(a, b, info);
                    }
                }
            }
        }
    }

    /**
     * Pushes overlapping bodies apart.
     *
     * Impulses fix velocity, but objects may still end up slightly inside each
     * other.
     * This method moves them apart based on penetration depth.
     *
     * Inverse mass controls how much each object moves. A lighter object moves
     * more,
     * while an infinite-mass object does not move at all.
     */
    private void positionalCorrection(PhysicsBody a, PhysicsBody b, CollisionInfo info) {
        double invMassA = a.getInverseMass();
        double invMassB = b.getInverseMass();
        double invMassSum = invMassA + invMassB;

        if (invMassSum == 0) {
            return;
        }

        /*
         * Slop allows a tiny bit of overlap so resting objects do not constantly
         * jitter. Correction percent prevents the engine from overcorrecting in one
         * frame.
         */
        double correctionMagnitude = Math.max(info.penetration - CORRECTION_SLOP, 0.0) / invMassSum
                * CORRECTION_PERCENT;

        Vector2 correction = info.normal.multiply(correctionMagnitude);

        if (a.getMassType() != MassType.INFINITE) {
            a.move(correction.multiply(-invMassA));
        }

        if (b.getMassType() != MassType.INFINITE) {
            b.move(correction.multiply(invMassB));
        }
    }

    /**
     * Applies the main collision impulse between two bodies.
     *
     * This changes the bodies' velocities so they stop moving into each other.
     * It also supports bounce through restitution and rotation through contact
     * points that are not at the center of mass.
     */
    private void applyCollisionImpulse(PhysicsBody a, PhysicsBody b, CollisionInfo info) {
        Vector2 contactPoint = info.contactPoint;
        Vector2 normal = info.normal;

        /*
         * rA and rB are offsets from each body's center to the collision point.
         * These are needed because off-center collisions create torque.
         */
        Vector2 rA = contactPoint.subtract(a.getPosition());
        Vector2 rB = contactPoint.subtract(b.getPosition());

        /*
         * Relative velocity at the contact point includes both linear velocity and
         * rotational velocity. This matters because a spinning object can have a
         * different velocity at the contact point than at its center.
         */
        Vector2 relativeVelocity = b.getVelocityAtPoint(contactPoint).subtract(a.getVelocityAtPoint(contactPoint));

        double velocityAlongNormal = relativeVelocity.dot(normal);

        /*
         * If velocityAlongNormal is positive, the bodies are already moving apart,
         * so applying another collision impulse would be wrong.
         */
        if (velocityAlongNormal > 0) {
            return;
        }

        /*
         * Use the smaller restitution so a very bouncy object does not make a dull
         * object bounce more than it should.
         */
        double restitution = Math.min(a.getFixture().getRestitution(), b.getFixture().getRestitution());

        if (Math.abs(velocityAlongNormal) < RESTITUTION_THRESHOLD) {
            restitution = 0.0;
        }

        /*
         * These cross products measure how much the collision normal can rotate
         * each object around its center.
         */
        double rACrossN = rA.cross(normal);
        double rBCrossN = rB.cross(normal);

        /*
         * The denominator combines linear mass and rotational inertia.
         *
         * If the collision happens far from the center, the rotational terms become
         * more important because some of the impulse turns into spin.
         */
        double denominator = a.getInverseMass() + b.getInverseMass()
                + (rACrossN * rACrossN) * a.getInverseInertia()
                + (rBCrossN * rBCrossN) * b.getInverseInertia();

        if (denominator == 0) {
            return;
        }

        /*
         * Impulse scalar controls how strong the impulse is.
         *
         * The negative sign flips the direction because we are cancelling motion
         * into the collision normal.
         */
        double impulseScalar = -(1.0 + restitution) * velocityAlongNormal / denominator;

        Vector2 impulse = normal.multiply(impulseScalar);

        /*
         * Equal and opposite impulses are applied to the two bodies.
         */
        a.applyImpulse(impulse.negate(), contactPoint);
        b.applyImpulse(impulse, contactPoint);

        applyFrictionImpulse(a, b, normal, contactPoint, rA, rB, impulseScalar);
    }

    /**
     * Applies friction at the collision point.
     *
     * The normal impulse handles bouncing/separating. Friction handles sideways
     * sliding along the collision surface.
     */
    private void applyFrictionImpulse(
            PhysicsBody a,
            PhysicsBody b,
            Vector2 normal,
            Vector2 contactPoint,
            Vector2 rA,
            Vector2 rB,
            double normalImpulseScalar) {

        Vector2 relativeVelocity = b.getVelocityAtPoint(contactPoint).subtract(a.getVelocityAtPoint(contactPoint));

        /*
         * Remove the normal part of the velocity. What remains is the tangent
         * direction, which represents sliding along the surface.
         */
        double velAlongNormal = relativeVelocity.dot(normal);
        Vector2 tangent = relativeVelocity.subtract(normal.multiply(velAlongNormal));

        if (tangent.magnitudeSquared() < 1e-10) {
            return;
        }

        tangent = tangent.normalized();

        double rACrossT = rA.cross(tangent);
        double rBCrossT = rB.cross(tangent);

        double denominator = a.getInverseMass() + b.getInverseMass()
                + (rACrossT * rACrossT) * a.getInverseInertia()
                + (rBCrossT * rBCrossT) * b.getInverseInertia();

        if (denominator == 0) {
            return;
        }

        double frictionImpulseScalar = -relativeVelocity.dot(tangent) / denominator;

        /*
         * Average the two friction values.
         *
         * The friction impulse is clamped so friction cannot become stronger than
         * the collision force itself. This is a simple Coulomb friction model.
         */
        double mu = (a.getFixture().getFriction() + b.getFixture().getFriction()) * 0.5;
        double maxFriction = Math.abs(normalImpulseScalar) * mu;

        frictionImpulseScalar = Math.max(-maxFriction, Math.min(frictionImpulseScalar, maxFriction));

        Vector2 frictionImpulse = tangent.multiply(frictionImpulseScalar);

        a.applyImpulse(frictionImpulse.negate(), contactPoint);
        b.applyImpulse(frictionImpulse, contactPoint);
    }
}