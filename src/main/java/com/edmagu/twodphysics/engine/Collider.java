package com.edmagu.twodphysics.engine;

public class Collider {

    // What kind of shape this collider represents.
    private ShapeType shapeType;

    // Circle data.
    private double radius;

    // Rectangle data.
    private double halfWidth;
    private double halfHeight;

    // Polygon data.
    private Vector2[] vertices;

    /**
     * Private constructor so colliders must be created through the factory methods.
     *
     * This keeps collider creation controlled. For example, a circle collider gets
     * a radius, a rectangle gets half-width/half-height, and a polygon gets vertices.
     */
    private Collider(ShapeType shapeType) {
        this.shapeType = shapeType;
    }

    /**
     * Creates a circle collider.
     */
    public static Collider circle(double radius) {
        Collider collider = new Collider(ShapeType.CIRCLE);
        collider.radius = radius;
        return collider;
    }

    /**
     * Creates a rectangle collider centered on the body.
     *
     * Half sizes are used because the rectangle extends equally from the center
     * in both directions.
     */
    public static Collider rectangle(double halfWidth, double halfHeight) {
        Collider collider = new Collider(ShapeType.RECTANGLE);
        collider.halfWidth = halfWidth;
        collider.halfHeight = halfHeight;
        return collider;
    }

    /**
     * Creates a polygon collider from local-space vertices.
     *
     * The vertices describe the shape relative to the body's center, not directly
     * in world coordinates.
     */
    public static Collider polygon(Vector2[] vertices) {
        Collider collider = new Collider(ShapeType.POLYGON);
        collider.vertices = vertices;
        return collider;
    }

    public ShapeType getShapeType() {
        return shapeType;
    }

    public double getRadius() {
        return radius;
    }

    public double getHalfWidth() {
        return halfWidth;
    }

    public double getHalfHeight() {
        return halfHeight;
    }

    public Vector2[] getVertices() {
        return vertices;
    }
}