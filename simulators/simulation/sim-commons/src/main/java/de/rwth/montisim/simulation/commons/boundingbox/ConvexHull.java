package de.rwth.montisim.simulation.commons.boundingbox;

import de.rwth.montisim.commons.utils.Vec3;

public class ConvexHull implements BoundingBox {
    // A point is outside if dot(point, halfSpaceNormal[i]) > halfSpacePos[i] for any i
    // (the point is "on the good side" of any half-space)

    /**
     * Each normal points "outside" of the hull and is associated with a 'halfSpacePos' with the same index.
     */
    public Vec3 halfSpaceNormals[];
    /**
     * Obtained through the dot product of any point in the plane with its corresponding normal.
     */
    public double halfSpacePos[];
    /**
     * The list of corner points of the hull. Used when testing a Separating Axis from an external axis.
     */
    public Vec3 corners[];

    @Override
    public boolean collidesWith(BoundingBox other) {
        return other.collidesWith(this);
    }

    @Override
    public boolean collidesWith(AABB other) {
        throw new UnimplementedCollisionException("ConvexHull-AABB");
    }

    @Override
    public boolean collidesWith(OBB other) {
        return CollisionTests.collides(other, this);
    }

    @Override
    public boolean collidesWith(ConvexHull other) {
        throw new IllegalStateException("Unimplemented ConvexHull-ConvexHull collision detection");
    }

    @Override
    public boolean collidesWith(Union other) {
        for (BoundingBox bb : other.boundingBoxes) {
            if (bb.collidesWith(this)) return true;
        }
        return false;
    }
}
