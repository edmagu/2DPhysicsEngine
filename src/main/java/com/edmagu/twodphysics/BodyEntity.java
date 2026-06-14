package com.edmagu.twodphysics;

import java.awt.Color;

import com.edmagu.twodphysics.engine.PhysicsBody;
import com.edmagu.twodphysics.engine.Vector2;

public class BodyEntity {

    // Unique name/id used to find this entity later.
    private final String id;

    // The actual physics object being simulated.
    private final PhysicsBody body;

    // Shape name used by the GUI/rendering layer.
    private final String shapeType;

    // Bounciness value stored for display or reference.
    private final double restitution;

    // Color used when drawing this body.
    private final Color color;

    // Whether the mouse can grab/select this entity.
    private final boolean grabbable;

    // Cached position from the physics body.
    private Vector2 positionSnapshot;

    // Cached rotation from the physics body.
    private double rotationSnapshot;

    /**
     * Wraps a PhysicsBody with extra information the engine and GUI need.
     *
     * PhysicsBody stores the actual simulation data, but BodyEntity stores the
     * identity, color, shape label, and whether the object can be interacted with.
     */
    public BodyEntity(
            String id,
            PhysicsBody body,
            String shapeType,
            double restitution,
            Color color,
            boolean grabbable
    ) {
        this.id = id;
        this.body = body;
        this.shapeType = shapeType;
        this.restitution = restitution;
        this.color = color;
        this.grabbable = grabbable;

        // Initialize the cached position and rotation immediately.
        refreshSnapshot();
    }

    /**
     * Copies the body's current position and rotation into snapshot fields.
     *
     * This is useful when the rendering layer wants a stable copy of the body's
     * transform instead of directly relying on live physics values.
     */
    public void refreshSnapshot() {
        this.positionSnapshot = body.getPosition();
        this.rotationSnapshot = body.getRotation();
    }

    public String getId() {
        return id;
    }

    public PhysicsBody getBody() {
        return body;
    }

    public String getShapeType() {
        return shapeType;
    }

    public double getRestitution() {
        return restitution;
    }

    public Color getColor() {
        return color;
    }

    public boolean isGrabbable() {
        return grabbable;
    }

    public Vector2 getPositionSnapshot() {
        return positionSnapshot;
    }

    public double getRotationSnapshot() {
        return rotationSnapshot;
    }
}