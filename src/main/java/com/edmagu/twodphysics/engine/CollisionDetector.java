package com.edmagu.twodphysics.engine;

public class CollisionDetector {

    /**
     * Simple true/false collision check.
     *
     * This uses getCollisionInfo() internally and only returns whether the bodies
     * are colliding. Use getCollisionInfo() directly when the normal, penetration,
     * or contact point is needed.
     */
    public static boolean isColliding(PhysicsBody a, PhysicsBody b) {
        return getCollisionInfo(a, b).colliding;
    }

    /**
     * Chooses the correct collision test based on the two collider shapes.
     *
     * Different shape combinations need different math. Circle-circle is simple,
     * while rectangles and polygons use SAT. Rectangles are converted into polygon
     * vertices so they can reuse the polygon collision code.
     */
    public static CollisionInfo getCollisionInfo(PhysicsBody a, PhysicsBody b) {
        ShapeType sa = a.getCollider().getShapeType();
        ShapeType sb = b.getCollider().getShapeType();

        if (sa == ShapeType.CIRCLE && sb == ShapeType.CIRCLE) {
            return circleCircle(a, b);
        }

        if (sa == ShapeType.RECTANGLE && sb == ShapeType.RECTANGLE) {
            return polygonPolygon(rectangleToWorldVertices(a), rectangleToWorldVertices(b), a, b);
        }

        if (sa == ShapeType.CIRCLE && sb == ShapeType.RECTANGLE) {
            return circlePolygon(a, rectangleToWorldVertices(b), b);
        }

        if (sa == ShapeType.RECTANGLE && sb == ShapeType.CIRCLE) {
            return flip(circlePolygon(b, rectangleToWorldVertices(a), a));
        }

        if (sa == ShapeType.POLYGON && sb == ShapeType.POLYGON) {
            return polygonPolygon(polygonToWorldVertices(a), polygonToWorldVertices(b), a, b);
        }

        if (sa == ShapeType.CIRCLE && sb == ShapeType.POLYGON) {
            return circlePolygon(a, polygonToWorldVertices(b), b);
        }

        if (sa == ShapeType.POLYGON && sb == ShapeType.CIRCLE) {
            return flip(circlePolygon(b, polygonToWorldVertices(a), a));
        }

        if (sa == ShapeType.RECTANGLE && sb == ShapeType.POLYGON) {
            return polygonPolygon(rectangleToWorldVertices(a), polygonToWorldVertices(b), a, b);
        }

        if (sa == ShapeType.POLYGON && sb == ShapeType.RECTANGLE) {
            return polygonPolygon(polygonToWorldVertices(a), rectangleToWorldVertices(b), a, b);
        }

        return CollisionInfo.none();
    }

    /**
     * Detects collision between two circles.
     *
     * Two circles collide if the distance between their centers is less than the
     * sum of their radii.
     */
    private static CollisionInfo circleCircle(PhysicsBody a, PhysicsBody b) {
        Vector2 delta = b.getPosition().subtract(a.getPosition());
        double distanceSquared = delta.magnitudeSquared();
        double radiusSum = a.getCollider().getRadius() + b.getCollider().getRadius();

        if (distanceSquared >= radiusSum * radiusSum) {
            return CollisionInfo.none();
        }

        double distance = Math.sqrt(distanceSquared);

        /*
         * If the circles are almost exactly on top of each other, there is no clear
         * direction between them. In that case, use a default normal pointing right.
         */
        Vector2 normal = distance < 1e-10 ? new Vector2(1, 0) : delta.multiply(1.0 / distance);

        double penetration = radiusSum - distance;

        // Approximate contact point on the surface of circle A facing circle B.
        Vector2 contactPoint = a.getPosition().add(normal.multiply(a.getCollider().getRadius()));

        return new CollisionInfo(true, normal, penetration, contactPoint);
    }

    /**
     * Detects collision between two polygons using the Separating Axis Theorem.
     *
     * SAT says two convex polygons are separated if there is at least one axis where
     * their projections do not overlap. If every tested axis overlaps, the polygons
     * are colliding.
     */
    private static CollisionInfo polygonPolygon(
            Vector2[] vertsA, Vector2[] vertsB,
            PhysicsBody bodyA, PhysicsBody bodyB) {

        CollisionInfo checkA = testAxes(vertsA, vertsB);
        if (!checkA.colliding) {
            return CollisionInfo.none();
        }

        CollisionInfo checkB = testAxes(vertsB, vertsA);
        if (!checkB.colliding) {
            return CollisionInfo.none();
        }

        /*
         * The collision normal should use the axis with the smallest overlap.
         * That gives the shallowest direction to separate the two objects.
         */
        CollisionInfo best = checkA.penetration <= checkB.penetration ? checkA : checkB;

        Vector2 centerDirection = bodyB.getPosition().subtract(bodyA.getPosition());

        /*
         * Make sure the normal points from body A toward body B.
         */
        Vector2 normal = best.normal.dot(centerDirection) < 0 ? best.normal.negate() : best.normal;

        Vector2 contactPoint = findContactPoint(vertsA, vertsB);

        return new CollisionInfo(true, normal, best.penetration, contactPoint);
    }

    /**
     * Finds an approximate contact point between two polygons.
     *
     * This checks vertices from each polygon against edges of the other polygon.
     * The closest vertex-edge pair gives a reasonable contact point for impulse
     * solving.
     */
    private static Vector2 findContactPoint(Vector2[] vertsA, Vector2[] vertsB) {
        double minDistSq = Double.POSITIVE_INFINITY;
        Vector2 contact = new Vector2(0, 0);

        for (Vector2 v : vertsA) {
            for (int i = 0; i < vertsB.length; i++) {
                Vector2 a = vertsB[i];
                Vector2 b = vertsB[(i + 1) % vertsB.length];

                Vector2 closest = closestPointOnSegment(v, a, b);
                double distSq = v.subtract(closest).magnitudeSquared();

                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    contact = v.add(closest).multiply(0.5);
                }
            }
        }

        for (Vector2 v : vertsB) {
            for (int i = 0; i < vertsA.length; i++) {
                Vector2 a = vertsA[i];
                Vector2 b = vertsA[(i + 1) % vertsA.length];

                Vector2 closest = closestPointOnSegment(v, a, b);
                double distSq = v.subtract(closest).magnitudeSquared();

                if (distSq < minDistSq) {
                    minDistSq = distSq;
                    contact = v.add(closest).multiply(0.5);
                }
            }
        }

        return contact;
    }

    /**
     * Returns the closest point on a line segment to point p.
     *
     * The projection value t tells where p lands on the segment:
     *
     * t = 0 means closest to endpoint a.
     * t = 1 means closest to endpoint b.
     * between 0 and 1 means closest to somewhere on the segment.
     */
    private static Vector2 closestPointOnSegment(Vector2 p, Vector2 a, Vector2 b) {
        Vector2 ab = b.subtract(a);
        double abLenSq = ab.magnitudeSquared();

        if (abLenSq < 1e-10) {
            return a;
        }

        double t = p.subtract(a).dot(ab) / abLenSq;
        t = Math.max(0.0, Math.min(1.0, t));

        return a.add(ab.multiply(t));
    }

    /**
     * Tests all edge normals from vertsA as possible separating axes.
     *
     * For each polygon edge, a perpendicular axis is created. Both polygons are
     * projected onto that axis. If their projections do not overlap, there is no
     * collision.
     */
    private static CollisionInfo testAxes(Vector2[] vertsA, Vector2[] vertsB) {
        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vector2 smallestAxis = null;

        for (int i = 0; i < vertsA.length; i++) {
            Vector2 p1 = vertsA[i];
            Vector2 p2 = vertsA[(i + 1) % vertsA.length];

            Vector2 edge = p2.subtract(p1);

            // Perpendicular to the edge; this is the axis SAT tests.
            Vector2 axis = new Vector2(-edge.y, edge.x).normalized();

            double[] projA = project(vertsA, axis);
            double[] projB = project(vertsB, axis);

            double overlap = Math.min(projA[1], projB[1]) - Math.max(projA[0], projB[0]);

            if (overlap <= 0) {
                return CollisionInfo.none();
            }

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestAxis = axis;
            }
        }

        // Contact point is filled in later by polygonPolygon().
        return new CollisionInfo(true, smallestAxis, smallestOverlap, new Vector2(0, 0));
    }

    /**
     * Detects collision between a circle and a polygon using SAT.
     *
     * This tests the polygon edge normals, then also tests the axis from the circle
     * center to the closest polygon vertex. That extra axis is needed for corner
     * collisions.
     */
    private static CollisionInfo circlePolygon(
            PhysicsBody circleBody, Vector2[] polygon, PhysicsBody polygonBody) {

        Vector2 center = circleBody.getPosition();
        double radius = circleBody.getCollider().getRadius();

        double smallestOverlap = Double.POSITIVE_INFINITY;
        Vector2 smallestAxis = null;

        /*
         * First test the normal of every polygon edge.
         */
        for (int i = 0; i < polygon.length; i++) {
            Vector2 p1 = polygon[i];
            Vector2 p2 = polygon[(i + 1) % polygon.length];

            Vector2 edge = p2.subtract(p1);
            Vector2 axis = new Vector2(-edge.y, edge.x).normalized();

            double[] projPoly = project(polygon, axis);

            double centerProj = center.dot(axis);
            double[] projCircle = { centerProj - radius, centerProj + radius };

            double overlap = Math.min(projPoly[1], projCircle[1]) - Math.max(projPoly[0], projCircle[0]);

            if (overlap <= 0) {
                return CollisionInfo.none();
            }

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestAxis = axis;
            }
        }

        /*
         * Then test the axis from the circle center to the closest polygon vertex.
         * Without this, circle-vs-corner collisions can be inaccurate.
         */
        Vector2 closest = closestVertex(center, polygon);
        Vector2 axisToVertex = closest.subtract(center);

        if (axisToVertex.magnitudeSquared() > 1e-10) {
            Vector2 axis = axisToVertex.normalized();

            double[] projPoly = project(polygon, axis);

            double centerProj = center.dot(axis);
            double[] projCircle = { centerProj - radius, centerProj + radius };

            double overlap = Math.min(projPoly[1], projCircle[1]) - Math.max(projPoly[0], projCircle[0]);

            if (overlap <= 0) {
                return CollisionInfo.none();
            }

            if (overlap < smallestOverlap) {
                smallestOverlap = overlap;
                smallestAxis = axis;
            }
        }

        Vector2 centerDirection = polygonBody.getPosition().subtract(circleBody.getPosition());

        /*
         * Make sure the normal points from the circle toward the polygon.
         */
        Vector2 normal = smallestAxis.dot(centerDirection) < 0 ? smallestAxis.negate() : smallestAxis;

        // Approximate contact point on the circle surface facing the polygon.
        Vector2 contactPoint = center.add(normal.multiply(radius));

        return new CollisionInfo(true, normal, smallestOverlap, contactPoint);
    }

    /**
     * Flips collision info when the original test was run in the opposite order.
     *
     * For example, circlePolygon(circle, polygon) returns a normal from the circle
     * to the polygon. If the caller asked for polygon-circle, the normal must be
     * reversed so it still points from A to B.
     */
    private static CollisionInfo flip(CollisionInfo info) {
        if (!info.colliding) {
            return info;
        }

        return new CollisionInfo(true, info.normal.negate(), info.penetration, info.contactPoint);
    }

    /**
     * Projects all polygon vertices onto an axis.
     *
     * The result is a min/max interval. SAT compares these intervals to decide
     * whether two shapes overlap along that axis.
     */
    private static double[] project(Vector2[] vertices, Vector2 axis) {
        double min = vertices[0].dot(axis);
        double max = min;

        for (int i = 1; i < vertices.length; i++) {
            double p = vertices[i].dot(axis);

            if (p < min) {
                min = p;
            } else if (p > max) {
                max = p;
            }
        }

        return new double[] { min, max };
    }

    /**
     * Finds the polygon vertex closest to a given point.
     *
     * Used by circle-polygon collision to create the extra axis needed for corner
     * cases.
     */
    private static Vector2 closestVertex(Vector2 point, Vector2[] vertices) {
        Vector2 closest = vertices[0];
        double closestDistSq = point.subtract(closest).magnitudeSquared();

        for (int i = 1; i < vertices.length; i++) {
            double d = point.subtract(vertices[i]).magnitudeSquared();

            if (d < closestDistSq) {
                closestDistSq = d;
                closest = vertices[i];
            }
        }

        return closest;
    }

    /**
     * Converts a rectangle body into world-space vertices.
     *
     * Rectangles are stored as half-width and half-height, but SAT needs the four
     * actual corners in world space.
     */
    private static Vector2[] rectangleToWorldVertices(PhysicsBody body) {
        double hw = body.getCollider().getHalfWidth();
        double hh = body.getCollider().getHalfHeight();

        Vector2[] local = {
                new Vector2(-hw, -hh),
                new Vector2(hw, -hh),
                new Vector2(hw, hh),
                new Vector2(-hw, hh)
        };

        return transformToWorld(local, body);
    }

    /**
     * Converts a polygon body's local vertices into world-space vertices.
     */
    private static Vector2[] polygonToWorldVertices(PhysicsBody body) {
        return transformToWorld(body.getCollider().getVertices(), body);
    }

    /**
     * Applies body rotation and position to local vertices.
     *
     * Local vertices describe the shape around the body center. This method rotates
     * those vertices by the body's current rotation and then translates them into
     * world space.
     */
    private static Vector2[] transformToWorld(Vector2[] local, PhysicsBody body) {
        Vector2[] world = new Vector2[local.length];

        double cos = Math.cos(body.getRotation());
        double sin = Math.sin(body.getRotation());
        Vector2 pos = body.getPosition();

        for (int i = 0; i < local.length; i++) {
            double x = local[i].x * cos - local[i].y * sin;
            double y = local[i].x * sin + local[i].y * cos;

            world[i] = new Vector2(pos.x + x, pos.y + y);
        }

        return world;
    }
}