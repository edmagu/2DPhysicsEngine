package com.edmagu.twodphysics.engine;

public class PhysicsBody {

    // Basic motion state
    private Vector2 position;
    private Vector2 velocity;
    private Vector2 force;

    // Rotation state
    private double rotation = 0.0;
    private double angularVelocity = 0.0;
    private double torque;

    // Mass properties
    private double mass;
    private double inverseMass;
    private double inertia;
    private double inverseInertia;

    // Collision and material data
    private MassType massType;
    private Collider collider;
    private PhysicsFixture fixture;

    // Used for fast-moving objects that need more accurate collision checks
    private boolean bullet;

    // Damping slows movement/rotation over time

    /**
     * Creates a physics body with a position, collider, mass type, and material
     * properties.
     *
     * The body starts with no velocity, force, torque, or rotation. After setting
     * up
     * the collider and fixture, the constructor calculates the mass and inertia
     * based
     * on the shape and density.
     */
    public PhysicsBody(Vector2 position, Collider collider, MassType massType, double density, double restitution) {
        this.position = position;
        this.velocity = Vector2.zero();
        this.force = Vector2.zero();

        this.rotation = 0;
        this.angularVelocity = 0;
        this.torque = 0;

        this.collider = collider;
        this.massType = massType;

        // Friction is hard-coded to 0.3 for now.
        this.fixture = new PhysicsFixture(density, restitution, 0.3);

        calculateMass();

        this.bullet = false;
    }

    /**
     * Calculates the body's mass and rotational inertia from its shape.
     *
     * Mass controls how strongly the body reacts to forces.
     * Inertia controls how strongly the body reacts to torque.
     *
     * The engine stores inverseMass and inverseInertia because physics formulas
     * use division by mass a lot. Multiplying by the inverse is cleaner and avoids
     * divide-by-zero issues for static bodies.
     */
    private void calculateMass() {

        /*
         * Infinite-mass bodies are treated as static.
         *
         * They cannot be moved by forces, impulses, or torque. Setting inverseMass
         * and inverseInertia to 0 makes the math naturally produce no movement:
         *
         * acceleration = force * inverseMass
         *
         * If inverseMass is 0, acceleration is 0.
         */
        if (massType == MassType.INFINITE) {
            this.mass = Double.POSITIVE_INFINITY;
            this.inverseMass = 0.0;

            this.inertia = Double.POSITIVE_INFINITY;
            this.inverseInertia = 0.0;

            return;
        }

        /*
         * Circle mass:
         *
         * area = πr²
         * mass = area * density
         *
         * Circle inertia uses the solid disk formula:
         *
         * I = 1/2 * m * r²
         */
        if (collider.getShapeType() == ShapeType.CIRCLE) {
            double r = collider.getRadius();
            double area = Math.PI * r * r;

            this.mass = area * fixture.getDensity();
            this.inertia = 0.5 * mass * r * r;
        }

        /*
         * Rectangle mass:
         *
         * The collider stores half-width and half-height, so the full size is
         * twice those values.
         *
         * area = width * height
         * mass = area * density
         *
         * Rectangle inertia around its center:
         *
         * I = m(w² + h²) / 12
         */
        else if (collider.getShapeType() == ShapeType.RECTANGLE) {
            double w = collider.getHalfWidth() * 2.0;
            double h = collider.getHalfHeight() * 2.0;
            double area = w * h;

            this.mass = area * fixture.getDensity();
            this.inertia = mass * (w * w + h * h) / 12.0;
        }

        /*
         * Fallback for unknown shapes.
         *
         * This prevents the body from breaking if a shape type is added later
         * but mass calculation has not been implemented for it yet.
         */
        else {
            this.mass = fixture.getDensity();
            this.inertia = mass;
        }

        // Prevent invalid mass values from causing division problems.
        if (mass <= 0) {
            mass = 1.0;
        }

        if (inertia <= 0) {
            inertia = 1.0;
        }

        this.inverseMass = 1.0 / mass;
        this.inverseInertia = 1.0 / inertia;
    }

    /**
     * Advances the body forward by one physics step.
     *
     * This uses simple Euler integration:
     *
     * acceleration changes velocity,
     * velocity changes position,
     * angular acceleration changes angular velocity,
     * angular velocity changes rotation.
     */
    public void update(double dt, Vector2 gravity) {

        // Static bodies do not move or rotate.
        if (massType == MassType.INFINITE) {
            return;
        }

        /*
         * Linear acceleration comes from gravity plus any forces applied this frame.
         *
         * force * inverseMass is the same as force / mass.
         * Gravity is already acceleration, so it can be added directly.
         */
        Vector2 acceleration = gravity.add(force.multiply(inverseMass));

        /*
         * Angular acceleration is the rotational version of regular acceleration.
         *
         * torque / inertia becomes torque * inverseInertia.
         */
        double angularAcceleration = torque * inverseInertia;

        // Apply acceleration to velocity.
        velocity = velocity.add(acceleration.multiply(dt));

        // Apply angular acceleration to angular velocity.
        angularVelocity += angularAcceleration * dt;

        /*
         * Damping reduces motion over time.
         *
         * This version is stable because it divides by:
         *
         * 1 + dt * damping
         *
         * instead of subtracting directly from velocity. That helps avoid weird
         * behavior where damping accidentally reverses the object.
         */

        // Move and rotate the body using the updated velocities.
        position = position.add(velocity.multiply(dt));
        rotation += angularVelocity * dt;

        /*
         * Forces and torque are cleared after every update.
         *
         * This means applyForce() and applyTorque() only affect the next physics
         * step. Continuous forces need to be applied every frame.
         */
        force = Vector2.zero();
        torque = 0;

        /*
         * Floating-point math can leave tiny leftover velocities.
         *
         * These values are too small to matter visually, but they can cause bodies
         * to drift forever. This snaps tiny movement to zero.
         */
        if (velocity.magnitudeSquared() < 1e-8 && Math.abs(angularVelocity) < 1e-8) {
            velocity = Vector2.zero();
            angularVelocity = 0;
        }
    }

    /**
     * Applies an instant impulse at a point on the body.
     *
     * Unlike force, impulse changes velocity immediately. This is usually used
     * for collisions, where the body needs to react instantly.
     *
     * If the impulse is applied away from the center, it also creates rotation.
     */
    public void applyImpulse(Vector2 impulse, Vector2 contactPoint) {

        if (massType == MassType.INFINITE) {
            return;
        }

        // Linear impulse changes the body's normal velocity.
        velocity = velocity.add(impulse.multiply(inverseMass));

        /*
         * r is the offset from the center of the body to the contact point.
         *
         * If the impulse hits the center, r is basically zero, so there is no spin.
         * If the impulse hits off-center, the cross product creates angular motion.
         */
        Vector2 r = contactPoint.subtract(position);

        /*
         * r.cross(impulse) gives the rotational effect of the impulse.
         *
         * A larger offset from the center creates more rotation.
         * A stronger impulse also creates more rotation.
         */
        angularVelocity += r.cross(impulse) * inverseInertia;
    }

    /**
     * Applies an instant impulse to the whole body without creating rotation.
     *
     * This is useful when you want to push the object in a direction but do not
     * care where the push happened.
     */
    public void applyLinearImpulse(Vector2 impulse) {

        if (massType == MassType.INFINITE) {
            return;
        }

        velocity = velocity.add(impulse.multiply(inverseMass));
    }

    /**
     * Applies a force through the center of the body.
     *
     * Since this force acts through the center, it affects movement but does not
     * create torque.
     */
    public void applyForce(Vector2 force) {
        this.force = this.force.add(force);
    }

    /**
     * Applies a force at a specific point in world space.
     *
     * This affects both movement and rotation. The force moves the body normally,
     * while the offset from the center creates torque.
     */
    public void applyForce(Vector2 force, Vector2 worldPoint) {

        this.force = this.force.add(force);

        /*
         * The farther the force is applied from the center, the more torque it can
         * create. This is why pushing the edge of an object spins it more than
         * pushing directly on its center.
         */
        Vector2 r = worldPoint.subtract(position);
        this.torque += r.cross(force);
    }

    /**
     * Applies rotational force directly.
     *
     * This changes angular acceleration during the next update.
     */
    public void applyTorque(double torque) {
        this.torque += torque;
    }

    /*
     * A getter is a method that returns a value from a class
     * and a setter is a method that sets a value in a class
     */
    // Position and velocity getters/setters
    public Vector2 getPosition() {
        return position;
    }

    public void setPosition(Vector2 position) {
        this.position = position;
    }

    public Vector2 getLinearVelocity() {
        return velocity;
    }

    public void setLinearVelocity(Vector2 velocity) {
        this.velocity = velocity;
    }

    public void setLinearVelocity(double x, double y) {
        this.velocity = new Vector2(x, y);
    }
    // Rotation getters/setters
    public double getRotation() {
        return rotation;
    }

    public void setRotation(double rotation) {
        this.rotation = rotation;
    }
    // Angular velocity getters/setters 
    public double getAngularVelocity() {
        return angularVelocity;
    }

    public void setAngularVelocity(double angularVelocity) {
        this.angularVelocity = angularVelocity;
    }
    // Mass and inertia getters
    public double getMass() {
        return mass;
    }

    public double getInverseMass() {
        return inverseMass;
    }

    public double getInertia() {
        return inertia;
    }

    public double getInverseInertia() {
        return inverseInertia;
    }

    public MassType getMassType() {
        return massType;
    }
    // Collider and fixture getters
    public Collider getCollider() {
        return collider;
    }

    public PhysicsFixture getFixture() {
        return fixture;
    }


    public void setBullet(boolean bullet) {
        this.bullet = bullet;
    }

    public boolean isBullet() {
        return bullet;
    }

    /**
     * Sets linear damping.
     *
     * Negative damping is blocked because it would add energy instead of removing
     * it,
     * making the object speed up over time.
     */

    /**
     * Sets angular damping.
     *
     * Negative angular damping is blocked for the same reason as linear damping.
     */

    /**
     * Directly shifts the body by a given amount.
     *
     * This is not a physics-based movement. It simply changes the position.
     */
    public void move(Vector2 amount) {
        this.position = this.position.add(amount);
    }

    /**
     * Returns the velocity of a specific point on the body.
     *
     * This matters for rotating bodies because a point near the edge can be moving
     * faster than the center. Collision solving needs this so it can calculate the
     * true velocity at the contact point.
     */
    public Vector2 getVelocityAtPoint(Vector2 worldPoint) {

        Vector2 r = worldPoint.subtract(position);

        /*
         * In 2D, rotational velocity at a point is perpendicular to the radius from
         * the center:
         *
         * r = (x, y)
         * rotational velocity = (-angularVelocity * y, angularVelocity * x)
         */
        Vector2 rotationalVelocity = new Vector2(
                -angularVelocity * r.y,
                angularVelocity * r.x);

        return velocity.add(rotationalVelocity);
    }
}