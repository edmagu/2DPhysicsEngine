

import java.awt.Color;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;


import engine.MassType;
import engine.PhysicsBody;
import engine.PhysicsFixture;
import engine.PhysicsWorld;
import engine.RopeJoint;
import engine.Vector2;

public class PhysicsEngine {

    // The main physics simulation world.
    private final PhysicsWorld world = new PhysicsWorld();

    // Stores all spawned bodies by ID while preserving insertion order.
    private final Map<String, BodyEntity> entities = new LinkedHashMap<>();

    // Main body used for testing constant force controls.
    private PhysicsBody primaryBody;

    // Whether the physics simulation is currently paused.
    private boolean paused = false;

    // Optional force applied every frame to the primary body.
    private Vector2 constantForce = null;

    // World settings controlled by the GUI.
    private static final double DEFAULT_FRICTION = 1.0;
    private static final double DEFAULT_GRAVITY_STRENGTH = 9.81;
    private static final double DEFAULT_AIR_RESISTANCE_MULTIPLIER = 1.0;

    /*
     * Air resistance is controlled as a multiplier.
     * A GUI value of 1.0 means:
     * linear damping = 1.0 * 0.25
     * angular damping = 1.0 * 0.10
     */
    private static final double BASE_LINEAR_DAMPING = 0.25;
    private static final double BASE_ANGULAR_DAMPING = 0.10;

    private double globalFriction = DEFAULT_FRICTION;
    private double gravityStrength = DEFAULT_GRAVITY_STRENGTH;
    private double airResistanceMultiplier = DEFAULT_AIR_RESISTANCE_MULTIPLIER;
    private double globalLinearDamping = BASE_LINEAR_DAMPING * DEFAULT_AIR_RESISTANCE_MULTIPLIER;
    private double globalAngularDamping = BASE_ANGULAR_DAMPING * DEFAULT_AIR_RESISTANCE_MULTIPLIER;

    // Default interaction tuning.
    private static final double GRAB_RADIUS = 2.5;
    private static final double SPRING_STIFFNESS = 5.0;
    private static final double CCD_SPEED_THRESHOLD = 60.0;

    // Used by rendering to smooth between physics steps.
    private volatile double interpolationAlpha = 0.2;

    // Used to generate unique IDs for new objects.
    private int entitySequence = 0;

    // Mouse interaction state.
    private Vector2 mousePosition = null;
    private boolean mouseDown = false;
    private PhysicsBody grabbedBody = null;
    private Vector2 grabPointLocal = null;

    // controls whether the mouse creates a spring to pull bodies around or places
    // static
    private MouseMode mouseMode = MouseMode.RAGDOLL;

    // Stores two selected bodies for actions that need a pair.
    private PhysicsBody selectedBodyA = null;
    private PhysicsBody selectedBodyB = null;

    // Rope placement state.
    private PhysicsBody ropeBodyA = null;
    private Vector2 ropePointA = null;
    private boolean ropePlacing = false;

    /**
     * Creates a new physics engine.
     *
     * The constructor sets up the world and immediately loads the default scene.
     * This means the simulation starts with walls, balls, and a box already
     * created.
     */
    public PhysicsEngine() {
        configureWorld();
        loadScene(defaultScene());
    }

    /**
     * Sets up global physics world settings.
     *
     * Right now this only sets gravity. The y-value is negative because the world
     * treats downward as negative y.
     */
    private void configureWorld() {
        world.setGravity(new Vector2(0.0, -gravityStrength));
    }

    /**
     * Loads a group of object definitions into the world.
     *
     * Each definition describes what should be created, but it is not an actual
     * physics body yet. spawn() converts each definition into a real body.
     */
    public void loadScene(List<PhysicsObjectDefinition> definitions) {
        for (PhysicsObjectDefinition definition : definitions) {
            spawn(definition);
        }
    }

    /**
     * Creates the default starting scene.
     *
     * The scene contains four infinite-mass boundaries and a few dynamic objects.
     * Infinite-mass objects act as static walls because they cannot be moved by
     * collisions or forces.
     */
    private List<PhysicsObjectDefinition> defaultScene() {

        return List.of(
                PhysicsObjectDefinition.rectangle("floor", 15.0, 1.0, 0.0, -1.5)
                        .massType(MassType.INFINITE)
                        .color(Color.GRAY)
                        .restitution(0.5),

                PhysicsObjectDefinition.rectangle("ceiling", 15.0, 1.0, 0.0, 7.5)
                        .massType(MassType.INFINITE)
                        .color(Color.GRAY)
                        .restitution(0.5),

                PhysicsObjectDefinition.rectangle("rightWall", 1.0, 10.0, 7.5, 3.0)
                        .massType(MassType.INFINITE)
                        .color(Color.GRAY)
                        .restitution(0.5),

                PhysicsObjectDefinition.rectangle("leftWall", 1.0, 10.0, -7.5, 3.0)
                        .massType(MassType.INFINITE)
                        .color(Color.GRAY)
                        .restitution(0.5),

                PhysicsObjectDefinition.circle("ball1", 1.0, -3.0, 2.0)
                        .density(2.59)
                        .restitution(0.75)
                        .angularVelocity(.0)
                        .color(Color.RED),

                PhysicsObjectDefinition.circle("ball2", 1.0, 3.0, 2.0)
                        .density(2.59)
                        .restitution(0.75)
                        .velocity(0.0, 0.0)
                        .color(Color.ORANGE),

                PhysicsObjectDefinition.rectangle("box1", 0.7, 0.7, 0.0, 4.5)
                        .density(1000.0)
                        .restitution(0.4)
                        .angularVelocity(5.0)
                        .color(Color.CYAN));
    }

    public PhysicsWorld getWorld() {
        return world;
    }

    public synchronized Collection<BodyEntity> getEntities() {
        return List.copyOf(this.entities.values());
    }

    public synchronized BodyEntity getEntity(String id) {
        return this.entities.get(id);
    }

    public void setConstantForce(Vector2 force) {
        this.constantForce = force;
    }

    public void clearConstantForce() {
        this.constantForce = null;
    }

    public synchronized void setMousePosition(Vector2 mousePosition) {
        this.mousePosition = mousePosition;
    }

    /**
     * Updates whether the mouse is currently pressed.
     *
     * When the mouse is pressed, the engine tries to select the nearest grabbable
     * body. The first click selects body A, the second different body selects body
     * B,
     * and clicking again restarts the selection.
     *
     * When the mouse is released, the active grab is cleared so the spring stops
     * pulling the body.
     */
    public synchronized void setMouseDown(boolean mouseDown) {
        this.mouseDown = mouseDown;

        if (mouseDown && this.mousePosition != null) {
            PhysicsBody clicked = findNearestBody(this.mousePosition, GRAB_RADIUS);

            if (clicked != null) {
                if (selectedBodyA == null) {
                    selectedBodyA = clicked;
                } else if (selectedBodyB == null && clicked != selectedBodyA) {
                    selectedBodyB = clicked;
                } else {
                    selectedBodyA = clicked;
                    selectedBodyB = null;
                }
            }
        }

        if (!mouseDown) {
            this.grabbedBody = null;
            this.grabPointLocal = null;
        }
    }

    /**
     * Creates a rope joint between two clicked bodies.
     *
     * The first click stores the first body and the local anchor point on that
     * body.
     * The second click finds another body and creates the actual rope between the
     * saved first anchor and the second clicked point.
     */
    public synchronized void bindRope(Vector2 mousePoint) {
        if (mousePoint == null) {
            return;
        }

        PhysicsBody clickedBody = findNearestBody(mousePoint, 2.0);
        if (clickedBody == null) {
            return;
        }

        if (!ropePlacing) {
            ropeBodyA = clickedBody;
            ropePointA = worldToLocal(clickedBody, mousePoint);
            ropePlacing = true;
            return;
        }

        Vector2 worldAnchorA = localToWorld(ropeBodyA, ropePointA);

        world.addRope(
                ropeBodyA,
                clickedBody,
                worldAnchorA,
                mousePoint);

        ropeBodyA = null;
        ropePointA = null;
        ropePlacing = false;
    }

    public synchronized void clearSelection() {
        selectedBodyA = null;
        selectedBodyB = null;
    }

    public synchronized List<RopeJoint> getRopes() {
        return new ArrayList<>(world.getRopes());
    }

    public synchronized void unrope() {
        world.clearRopes();
    }

    /**
     * Updates the whole physics engine by one time step.
     *
     * First, it applies any constant force to the primary body. Then it marks fast
     * objects as bullets for better collision handling. After that, the world runs
     * its physics update.
     *
     * The mouse spring is passed into world.update() as a callback so it gets
     * applied
     * during the physics step rather than randomly outside the simulation update.
     */
    public synchronized void update(double dt) {
        /*
         * Pause stops the normal world simulation.
         *
         * However, mouse interaction is still allowed:
         *
         * STATIC mode directly places the object under the mouse.
         * RAGDOLL mode applies spring force and updates only the grabbed body.
         */
        if (paused) {
            if (mouseMode == MouseMode.STATIC) {
                updateStaticMouseDrag();
            } else if (mouseMode == MouseMode.RAGDOLL) {
                updatePausedRagdollDrag(dt);
            }

            for (BodyEntity entity : this.entities.values()) {
                entity.refreshSnapshot();
            }

            return;
        }

        if (this.constantForce != null && this.primaryBody != null) {
            this.primaryBody.applyForce(this.constantForce);
        }

        updateFastBodyBullets();

        if (mouseMode == MouseMode.RAGDOLL) {
            world.update(dt, () -> {
                updateMouseSpring();
            });
        } else {
            world.update(dt, null);
            updateStaticMouseDrag();
        }

        applyAirResistance(dt);

        for (BodyEntity entity : this.entities.values()) {
            entity.refreshSnapshot();
        }
    }

    /**
     * Applies global air resistance after the physics step.
     *
     * The GUI value is a multiplier:
     * 1.0 = 0.25 linear damping and 0.10 angular damping.
     * 1.5 = 0.375 linear damping and 0.15 angular damping.
     */
    private void applyAirResistance(double dt) {
        if (dt <= 0.0) {
            return;
        }

        double linearFactor = Math.max(0.0, 1.0 - globalLinearDamping * dt);
        double angularFactor = Math.max(0.0, 1.0 - globalAngularDamping * dt);

        for (PhysicsBody body : this.world.getBodies()) {
            if (body.getMassType() == MassType.INFINITE) {
                continue;
            }

            body.setLinearVelocity(body.getLinearVelocity().multiply(linearFactor));
            body.setAngularVelocity(body.getAngularVelocity() * angularFactor);
        }
    }

    /**
     * Marks very fast dynamic bodies as bullets.
     *
     * Bullet mode is meant to help with tunneling, where fast objects pass through
     * walls because they move too far in one frame. Static bodies do not need
     * bullet
     * mode, so they are always set to false.
     */
    private void updateFastBodyBullets() {
        for (PhysicsBody body : this.world.getBodies()) {
            if (body.getMassType() == MassType.INFINITE) {
                body.setBullet(false);
                continue;
            }

            double speed = body.getLinearVelocity().magnitude();
            body.setBullet(speed >= CCD_SPEED_THRESHOLD);
        }
    }

    /**
     * Pulls the grabbed body toward the mouse using a spring-like force.
     *
     * This does not teleport the body. Instead, it applies a force based on how far
     * the grab point is from the mouse. The farther the mouse is from the grab
     * point,
     * the stronger the pull.
     *
     * Because the force is applied at the grabbed point instead of the center, it
     * can
     * also create torque and rotate the body naturally.
     */
    private void updateMouseSpring() {
        if (!this.mouseDown || this.mousePosition == null) {
            return;
        }

        /*
         * If nothing is grabbed yet, find the nearest body and store the mouse point
         * in the body's local space. This makes the grab point stay attached to the
         * same part of the shape even if the body rotates.
         */
        if (this.grabbedBody == null) {
            this.grabbedBody = findNearestBody(this.mousePosition, GRAB_RADIUS);

            if (this.grabbedBody != null) {
                this.grabPointLocal = worldToLocal(this.grabbedBody, this.mousePosition);
            }
        }

        if (this.grabbedBody == null || this.grabPointLocal == null) {
            return;
        }

        Vector2 grabPointWorld = localToWorld(this.grabbedBody, this.grabPointLocal);

        double dx = this.mousePosition.x - grabPointWorld.x;
        double dy = this.mousePosition.y - grabPointWorld.y;

        double bodyMass = this.grabbedBody.getMass();
        double safeMass = Math.max(1e-6, bodyMass);
        double scale = SPRING_STIFFNESS * safeMass;

        Vector2 force = new Vector2(dx * scale, dy * scale);

        this.grabbedBody.applyForce(force, grabPointWorld);
    }

    /**
     * Allows ragdoll mouse dragging while the simulation is paused.
     *
     * Normal pause stops the whole world from updating. That means spring forces
     * would normally do nothing, because forces only affect motion during update().
     *
     * This method solves that by updating only the currently grabbed body with
     * zero gravity. The rest of the world stays frozen.
     */
    private void updatePausedRagdollDrag(double dt) {
        if (!this.mouseDown || this.mousePosition == null) {
            return;
        }

        /*
         * Use a few mini-steps so the paused dragging still feels smooth and stable.
         * updateMouseSpring() must run every mini-step because PhysicsBody.update()
         * clears forces after applying them.
         */
        int steps = 8;
        double subDt = dt / steps;

        for (int i = 0; i < steps; i++) {
            updateMouseSpring();

            if (this.grabbedBody != null) {
                /*
                 * Zero gravity prevents the object from falling while paused.
                 * Only the mouse spring force moves it.
                 */
                this.grabbedBody.update(subDt, Vector2.zero());
            }
        }
    }

    public synchronized void rotateGrabbedBody(double radians) {
        /*
         * Static rotation only makes sense when an object is being controlled.
         */
        if (mouseMode != MouseMode.STATIC) {
            return;
        }

        /*
         * If the mouse is down but grabbedBody has not been assigned yet,
         * try to grab the nearest body first.
         */
        if (grabbedBody == null && mousePosition != null) {
            grabbedBody = findNearestBody(mousePosition, GRAB_RADIUS);

            if (grabbedBody != null) {
                grabPointLocal = worldToLocal(grabbedBody, mousePosition);
            }
        }

        if (grabbedBody == null) {
            return;
        }

        grabbedBody.setRotation(grabbedBody.getRotation() + radians);
        grabbedBody.setAngularVelocity(0);
    }

    public synchronized void setMouseMode(MouseMode mouseMode) {
        if (mouseMode == null) {
            return;
        }

        this.mouseMode = mouseMode;

        // Clear the current grab so switching modes does not carry old grab state.
        this.grabbedBody = null;
        this.grabPointLocal = null;
    }

    public synchronized MouseMode getMouseMode() {
        return mouseMode;
    }

    public synchronized void toggleMouseMode() {
        if (mouseMode == MouseMode.RAGDOLL) {
            setMouseMode(MouseMode.STATIC);
        } else {
            setMouseMode(MouseMode.RAGDOLL);
        }
    }

    private void updateStaticMouseDrag() {
        if (!this.mouseDown || this.mousePosition == null) {
            return;
        }

        /*
         * Static mode grabs the nearest body, but instead of applying a force,
         * it directly controls the body's transform.
         */
        if (this.grabbedBody == null) {
            this.grabbedBody = findNearestBody(this.mousePosition, GRAB_RADIUS);

            if (this.grabbedBody != null) {
                this.grabPointLocal = worldToLocal(this.grabbedBody, this.mousePosition);
            }
        }

        if (this.grabbedBody == null || this.grabPointLocal == null) {
            return;
        }

        /*
         * Convert the local grab point back into a rotated world offset.
         * This lets the exact point you clicked stay under the mouse, even after
         * rotating with Q and E.
         */
        double cos = Math.cos(this.grabbedBody.getRotation());
        double sin = Math.sin(this.grabbedBody.getRotation());

        double offsetX = grabPointLocal.x * cos - grabPointLocal.y * sin;
        double offsetY = grabPointLocal.x * sin + grabPointLocal.y * cos;

        Vector2 newBodyPosition = new Vector2(
                mousePosition.x - offsetX,
                mousePosition.y - offsetY);

        this.grabbedBody.setPosition(newBodyPosition);

        /*
         * Since this is direct mouse control, stop physics velocity from fighting
         * the mouse. Otherwise the body could keep drifting or spinning after being
         * moved.
         */
        this.grabbedBody.setLinearVelocity(Vector2.zero());
        this.grabbedBody.setAngularVelocity(0);
    }

    /**
     * Finds the nearest grabbable body within a maximum distance.
     *
     * This is used for mouse grabbing, selecting bodies, removing bodies, and rope
     * placement. It ignores non-grabbable entities so static scene objects like
     * walls
     * are not accidentally selected.
     */
    private PhysicsBody findNearestBody(Vector2 point, double maxDistance) {
        PhysicsBody nearest = null;
        double nearestDistance = maxDistance;

        for (BodyEntity entity : this.entities.values()) {
            if (entity == null || !entity.isGrabbable()) {
                continue;
            }

            PhysicsBody candidate = entity.getBody();
            Vector2 candidatePosition = candidate.getPosition();

            double dx = point.x - candidatePosition.x;
            double dy = point.y - candidatePosition.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            if (distance <= nearestDistance) {
                nearestDistance = distance;
                nearest = candidate;
            }
        }

        return nearest;
    }

    /**
     * Pauses the simulation.
     *
     * While paused, the physics world stops updating. Bodies will stay frozen
     * in their current positions until the simulation is continued.
     */
    public synchronized void pauseSimulation() {
        this.paused = true;
    }

    /**
     * Continues the simulation after it has been paused.
     */
    public synchronized void continueSimulation() {
        this.paused = false;
    }

    /**
     * Switches between paused and running.
     *
     * This is useful for a single keybind like Backspace, where pressing it once
     * pauses and pressing it again continues.
     */
    public synchronized void togglePause() {
        this.paused = !this.paused;
    }

    /**
     * Returns whether the simulation is currently paused.
     */
    public synchronized boolean isPaused() {
        return paused;
    }

    /**
     * Resets the simulation back to the default scene.
     *
     * This clears all current bodies, ropes, selections, mouse grabs, and generated
     * entities. Then it reloads the same default scene created when the program
     * starts.
     */
    public synchronized void resetSimulation() {
        this.paused = false;

        // Remove all physics objects and ropes from the world.
        world.clear();

        // Remove all entity wrappers used by the GUI/engine.
        entities.clear();

        // Reset engine-controlled references.
        primaryBody = null;
        constantForce = null;

        // Reset object ID generation.
        entitySequence = 0;

        // Clear mouse interaction state.
        mouseDown = false;
        grabbedBody = null;
        grabPointLocal = null;

        // Clear selections.
        selectedBodyA = null;
        selectedBodyB = null;

        // Clear rope placement state.
        ropeBodyA = null;
        ropePointA = null;
        ropePlacing = false;

        // Reapply world settings and reload the default objects.
        configureWorld();
        loadScene(defaultScene());
    }

    /**
     * Creates a real body from a PhysicsObjectDefinition and adds it to the engine.
     *
     * The definition acts like a blueprint. Depending on the shape type, this
     * method
     * asks ShapeFactory to create the correct PhysicsBody. Then it applies the
     * stored velocity, angular velocity, visual info, and metadata.
     */
    public synchronized BodyEntity spawn(PhysicsObjectDefinition definition) {
        if (definition == null) {
            throw new IllegalArgumentException("definition cannot be null");
        }


        PhysicsBody body;

        switch (definition.shapeType) {
            case CIRCLE:
                body = ShapeFactory.createCircleBody(
                        definition.radius,
                        definition.x,
                        definition.y,
                        definition.massType,
                        definition.restitution,
                        definition.density);
                break;

            case RECTANGLE:
                body = ShapeFactory.createRectangleBody(
                        definition.halfWidth,
                        definition.halfHeight,
                        definition.x,
                        definition.y,
                        definition.massType,
                        definition.restitution,
                        definition.density);
                break;

            case POLYGON:
                body = ShapeFactory.createPolygonBody(
                        definition.vertices,
                        definition.x,
                        definition.y,
                        definition.massType,
                        definition.restitution,
                        definition.density);
                break;

            default:
                throw new IllegalArgumentException("Unsupported shape type: " + definition.shapeType);
        }

        body.setLinearVelocity(definition.velocity); // sets the linear velocity of the body that is passed through the class
        body.setAngularVelocity(definition.angularVelocity); // sets the angular velocity of the body that is passed through the class
        addBodyWithDefaults(body);

        BodyEntity entity = new BodyEntity(
                definition.id,
                body,
                definition.shapeType.name().toLowerCase(),
                definition.restitution,
                definition.color,
                definition.grabbable);

        this.entities.put(entity.getId(), entity);

        /*
         * The first non-static body becomes the primary body.
         *
         * This gives the engine one default object to apply constant forces to.
         */
        if (this.primaryBody == null && definition.massType != MassType.INFINITE) {
            this.primaryBody = body;
        }

        return entity;
    }

    /**
     * Applies engine-wide defaults before adding a body to the world.
     *
     * Right now this mainly sets fixture friction. Keeping this in one place makes
     * it
     * easier to change default physics behavior later.
     */
    private void addBodyWithDefaults(PhysicsBody body) {
        if (body == null) {
            return;
        }

        PhysicsFixture fixture = body.getFixture();
        if (fixture != null) {
            fixture.setFriction(globalFriction);
        }

        world.addBody(body);
    }

    /**
     * Removes an entity and cleans up anything that was pointing to its body.
     *
     * This avoids dangling references. For example, if the removed body was
     * grabbed,
     * selected, used for rope placement, or set as the primary body, those
     * references
     * are cleared.
     */
    public synchronized boolean removeEntity(String id) {
        BodyEntity entity = this.entities.remove(id);

        if (entity == null) {
            return false;
        }

        PhysicsBody body = entity.getBody();

        if (body == primaryBody) {
            primaryBody = null;
        }

        if (body == grabbedBody) {
            grabbedBody = null;
            grabPointLocal = null;
        }

        if (body == selectedBodyA) {
            selectedBodyA = null;
        }

        if (body == selectedBodyB) {
            selectedBodyB = null;
        }

        if (body == ropeBodyA) {
            ropeBodyA = null;
            ropePointA = null;
            ropePlacing = false;
        }

        world.removeRopesConnectedTo(body);
        return world.removeBody(body);
    }

    /**
     * Removes the nearest grabbable entity to a point.
     *
     * This works almost the same as findNearestBody(), but it needs the BodyEntity
     * instead of only the PhysicsBody because removal is done by entity ID.
     */
    public synchronized boolean removeNearestEntity(Vector2 point, double maxDistance) {
        if (point == null) {
            return false;
        }

        BodyEntity nearestEntity = null;
        double nearestDistance = maxDistance;

        for (BodyEntity entity : this.entities.values()) {
            if (entity == null || !entity.isGrabbable()) {
                continue;
            }

            PhysicsBody candidate = entity.getBody();
            Vector2 candidatePosition = candidate.getPosition();

            double dx = point.x - candidatePosition.x;
            double dy = point.y - candidatePosition.y;
            double distance = Math.sqrt(dx * dx + dy * dy);

            if (distance <= nearestDistance) {
                nearestDistance = distance;
                nearestEntity = entity;
            }
        }

        if (nearestEntity == null) {
            return false;
        }

        return removeEntity(nearestEntity.getId());
    }

    /**
     * Checks whether two stored bodies are currently colliding.
     */
    public boolean isColliding(String idA, String idB) {
        BodyEntity a = this.entities.get(idA);
        BodyEntity b = this.entities.get(idB);

        if (a == null || b == null) {
            return false;
        }

        return world.isInContact(a.getBody(), b.getBody());
    }

    /**
     * Converts a world-space point into the body's local space.
     *
     * Local space means the point is measured relative to the body's center and
     * rotation. This is useful for anchors because the anchor should stay attached
     * to the same part of the body even while the body moves or rotates.
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
     * Converts a body-local point back into world space.
     *
     * This is the opposite of worldToLocal(). It rotates the point by the body's
     * current rotation, then offsets it by the body's current position.
     */
    private Vector2 localToWorld(PhysicsBody body, Vector2 localPoint) {
        double cos = Math.cos(body.getRotation());
        double sin = Math.sin(body.getRotation());

        return body.getPosition().add(new Vector2(
                localPoint.x * cos - localPoint.y * sin,
                localPoint.x * sin + localPoint.y * cos));
    }



    /**
     * Generates a simple unique ID using a prefix and increasing number.
     */
    private String nextEntityId(String prefix) {
        this.entitySequence++;
        return prefix + this.entitySequence;
    }

    public BodyEntity spawnCircle(double radius, double x, double y) {
        return spawnCircle(radius, x, y, Vector2.zero(), 1.0, 0.5, Color.WHITE, false);
    }

    /**
     * Convenience method for creating a circle without manually building a
     * definition.
     */
    public BodyEntity spawnCircle(
            double radius,
            double x,
            double y,
            Vector2 velocity,
            double density,
            double restitution,
            Color color,
            boolean infiniteMass) {

        PhysicsObjectDefinition definition = PhysicsObjectDefinition.circle(nextEntityId("circle"), radius, x, y)
                .velocity(velocity.x, velocity.y)
                .density(density)
                .restitution(restitution)
                .color(color)
                .massType(infiniteMass ? MassType.INFINITE : MassType.NORMAL);

        return spawn(definition);
    }

    public BodyEntity spawnRectangle(double halfWidth, double halfHeight, double x, double y) {
        return spawnRectangle(halfWidth, halfHeight, x, y, Vector2.zero(), 1.0, 0.5, Color.WHITE, false);
    }

    /**
     * Convenience method for creating a rectangle without manually building a
     * definition.
     */
    public BodyEntity spawnRectangle(
            double halfWidth,
            double halfHeight,
            double x,
            double y,
            Vector2 velocity,
            double density,
            double restitution,
            Color color,
            boolean infiniteMass) {

        PhysicsObjectDefinition definition = PhysicsObjectDefinition
                .rectangle(nextEntityId("rectangle"), halfWidth, halfHeight, x, y)
                .velocity(velocity.x, velocity.y)
                .density(density)
                .restitution(restitution)
                .color(color)
                .massType(infiniteMass ? MassType.INFINITE : MassType.NORMAL);

        return spawn(definition);
    }

    /**
     * Creates a triangle by defining three polygon vertices.
     *
     * The top vertex is placed at positive halfHeight, while the bottom two
     * vertices
     * form the base. The triangle is then spawned as a polygon.
     */
    public BodyEntity spawnTriangle(
            double halfWidth,
            double halfHeight,
            double x,
            double y,
            Vector2 velocity,
            double density,
            double restitution,
            Color color,
            boolean infiniteMass) {

        Vector2[] vertices = new Vector2[] {
                new Vector2(0.0, halfHeight),
                new Vector2(-halfWidth, -halfHeight),
                new Vector2(halfWidth, -halfHeight)
        };

        return spawnPolygon(vertices, x, y, velocity, density, restitution, color, infiniteMass); // call spawnPolygon() to create the triangle as a polygon
    }

    public BodyEntity spawnPolygon(Vector2[] vertices, double x, double y) {
        return spawnPolygon(vertices, x, y, Vector2.zero(), 1.0, 0.5, Color.WHITE, false);
    }

    /**
     * Convenience method for creating a polygon from a vertex array.
     */
    public BodyEntity spawnPolygon(
            Vector2[] vertices,
            double x,
            double y,
            Vector2 velocity,
            double density,
            double restitution,
            Color color,
            boolean infiniteMass) {

        PhysicsObjectDefinition definition = PhysicsObjectDefinition.polygon(nextEntityId("polygon"), vertices, x, y)
                .velocity(velocity.x, velocity.y)
                .density(density)
                .restitution(restitution)
                .color(color)
                .massType(infiniteMass ? MassType.INFINITE : MassType.NORMAL);

        return spawn(definition);
    }

    public synchronized double getGlobalFriction() {
        return globalFriction;
    }

    /**
     * Changes friction for every existing fixture and for every body spawned later.
     */
    public synchronized void setGlobalFriction(double friction) {
        this.globalFriction = Math.max(0.0, friction);

        for (PhysicsBody body : this.world.getBodies()) {
            PhysicsFixture fixture = body.getFixture();

            if (fixture != null) {
                fixture.setFriction(this.globalFriction);
            }
        }
    }

    public synchronized double getGravityStrength() {
        return gravityStrength;
    }

    /**
     * Positive values pull downward because this world uses negative y as down.
     */
    public synchronized void setGravityStrength(double gravityStrength) {
        this.gravityStrength = Math.max(0.0, gravityStrength);
        world.setGravity(new Vector2(0.0, -this.gravityStrength));
    }

    public synchronized double getAirResistanceMultiplier() {
        return airResistanceMultiplier;
    }

    public synchronized void setAirResistanceMultiplier(double multiplier) {
        this.airResistanceMultiplier = Math.max(0.0, multiplier);
        this.globalLinearDamping = BASE_LINEAR_DAMPING * this.airResistanceMultiplier;
        this.globalAngularDamping = BASE_ANGULAR_DAMPING * this.airResistanceMultiplier;
    }

    public synchronized double getGlobalLinearDamping() {
        return globalLinearDamping;
    }

    public synchronized double getGlobalAngularDamping() {
        return globalAngularDamping;
    }

    public enum ShapeType {
        CIRCLE,
        RECTANGLE,
        POLYGON
    }

    public enum MouseMode {
        RAGDOLL,
        STATIC
    }

    /**
     * A small builder-style object used to describe a body before it is spawned.
     *
     * This keeps object creation readable. Instead of passing a huge constructor
     * with every possible value, the code can chain methods like:
     *
     * PhysicsObjectDefinition.circle(...)
     * .density(...)
     * .restitution(...)
     * .color(...);
     */
    public static class PhysicsObjectDefinition {

        // Required identity and shape type.
        private final String id;
        private final ShapeType shapeType;

        // Shape-specific data.
        private double radius;
        private double halfWidth;
        private double halfHeight;
        private Vector2[] vertices;

        // Spawn position.
        private double x;
        private double y;

        // Optional physics and render settings.
        private Vector2 velocity = new Vector2(0.0, 0.0);
        private double angularVelocity = 0.0;
        private MassType massType = MassType.NORMAL;
        private double restitution = 0.5;
        private double density = 1.0;
        private Color color = Color.WHITE;
        private boolean grabbable = true;

        /**
         * Private constructor used by the static factory methods.
         *
         * The constructor validates shared data, while circle(), rectangle(), and
         * polygon() fill in the shape-specific fields.
         */
        private PhysicsObjectDefinition(String id, ShapeType shapeType, double x, double y) {
            if (id == null || id.isBlank()) {
                throw new IllegalArgumentException("id cannot be null or blank");
            }

            this.id = id;
            this.shapeType = shapeType;
            this.x = x;
            this.y = y;
        }

        public static PhysicsObjectDefinition circle(String id, double radius, double x, double y) {
            PhysicsObjectDefinition definition = new PhysicsObjectDefinition(id, ShapeType.CIRCLE, x, y);
            definition.radius = radius;
            return definition;
        }

        public static PhysicsObjectDefinition rectangle(String id, double halfWidth, double halfHeight, double x,
                double y) {
            PhysicsObjectDefinition definition = new PhysicsObjectDefinition(id, ShapeType.RECTANGLE, x, y);
            definition.halfWidth = halfWidth;
            definition.halfHeight = halfHeight;
            return definition;
        }

        public static PhysicsObjectDefinition polygon(String id, Vector2[] vertices, double x, double y) {
            PhysicsObjectDefinition definition = new PhysicsObjectDefinition(id, ShapeType.POLYGON, x, y);
            definition.vertices = vertices;
            return definition;
        }

        public PhysicsObjectDefinition velocity(double vx, double vy) {
            this.velocity = new Vector2(vx, vy);
            return this;
        }

        public PhysicsObjectDefinition angularVelocity(double angularVelocity) {
            this.angularVelocity = angularVelocity;
            return this;
        }

        public PhysicsObjectDefinition massType(MassType massType) {
            this.massType = massType;
            return this;
        }

        public PhysicsObjectDefinition restitution(double restitution) {
            this.restitution = restitution;
            return this;
        }

        public PhysicsObjectDefinition density(double density) {
            this.density = density;
            return this;
        }

        public PhysicsObjectDefinition color(Color color) {
            this.color = color;
            return this;
        }

        public PhysicsObjectDefinition grabbable(boolean grabbable) {
            this.grabbable = grabbable;
            return this;
        }
    }

    public synchronized Vector2 getMousePosition() {
        return mousePosition;
    }

    public synchronized boolean isRopePlacing() {
        return ropePlacing;
    }

    /**
     * Returns the first rope anchor in world space while placing a rope.
     *
     * The anchor is stored locally so it stays attached to the body. When the GUI
     * needs to draw it, this method converts it back to world space.
     */
    public synchronized Vector2 getRopePointA() {
        if (ropeBodyA == null || ropePointA == null) {
            return null;
        }

        return localToWorld(ropeBodyA, ropePointA);
    }

    public void setInterpolationAlpha(double alpha) {
        this.interpolationAlpha = alpha;
    }

    public double getInterpolationAlpha() {
        return interpolationAlpha;
    }
}