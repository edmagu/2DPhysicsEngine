

import engine.Collider;
import engine.MassType;
import engine.PhysicsBody;
import engine.Vector2;

public class ShapeFactory {

    /**
     * Creates a circular physics body.
     *
     * This method keeps object creation cleaner by hiding the collider setup.
     * Code outside this class only needs to say what kind of body it wants,
     * instead of manually creating the collider every time.
     */
    public static PhysicsBody createCircleBody(
            double radius,
            double x,
            double y,
            MassType massType,
            double restitution,
            double density
    ) {
        Collider collider = Collider.circle(radius);

        return new PhysicsBody(
                new Vector2(x, y),
                collider,
                massType,
                density,
                restitution
        );
    }

    /**
     * Creates a rectangular physics body.
     *
     * The rectangle uses half-width and half-height because many physics engines
     * define box shapes around their center. That makes rotation and collision math
     * easier, since the object is centered at its position.
     */
    public static PhysicsBody createRectangleBody(
            double halfWidth,
            double halfHeight,
            double x,
            double y,
            MassType massType,
            double restitution,
            double density
    ) {
        Collider collider = Collider.rectangle(halfWidth, halfHeight);

        return new PhysicsBody(
                new Vector2(x, y),
                collider,
                massType,
                density,
                restitution
        );
    }

    /**
     * Creates a polygon physics body.
     *
     * The vertices are expected to be local-space points, meaning they describe
     * the shape around the body's own center. The body position then places the
     * whole polygon into the world.
     */
    public static PhysicsBody createPolygonBody(
            Vector2[] vertices,
            double x,
            double y,
            MassType massType,
            double restitution,
            double density
    ) {
        Collider collider = Collider.polygon(vertices);

        return new PhysicsBody(
                new Vector2(x, y),
                collider,
                massType,
                density,
                restitution
        );
    }
}