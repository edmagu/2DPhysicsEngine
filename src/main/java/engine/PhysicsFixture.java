package engine;

public class PhysicsFixture {

    // Used to calculate mass from shape area.
    private double density;

    // Bounciness during collisions.
    private double restitution;

    // Surface friction used during collision response.
    private double friction;

    /**
     * Stores the physical material values attached to a body.
     *
     * The body itself handles position, velocity, mass, and rotation.
     * The fixture stores how the body should behave as a material: how dense,
     * bouncy, and rough it is.
     */
    public PhysicsFixture(double density, double restitution, double friction) {
        this.density = density;
        this.restitution = restitution;
        this.friction = friction;
    }

    public double getDensity() {
        return density;
    }

    public double getRestitution() {
        return restitution;
    }

    public double getFriction() {
        return friction;
    }

    public void setDensity(double density) {
        this.density = density;
    }

    public void setRestitution(double restitution) {
        this.restitution = restitution;
    }

    public void setFriction(double friction) {
        this.friction = friction;
    }
}