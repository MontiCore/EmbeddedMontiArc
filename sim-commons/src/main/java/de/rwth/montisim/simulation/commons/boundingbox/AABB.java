/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.boundingbox;

import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;

@Typed("aabb")
public class AABB implements BoundingBox {
    public Vec3 min = new Vec3();
    public Vec3 max = new Vec3();

    public void init() {
        min.x = Double.POSITIVE_INFINITY;
        min.y = Double.POSITIVE_INFINITY;
        min.z = Double.POSITIVE_INFINITY;
        max.x = Double.NEGATIVE_INFINITY;
        max.y = Double.NEGATIVE_INFINITY;
        max.z = Double.NEGATIVE_INFINITY;
    }

    public void include(Vec3 point) {
        min.x = Math.min(min.x, point.x);
        max.x = Math.max(max.x, point.x);
        min.y = Math.min(min.y, point.y);
        max.y = Math.max(max.y, point.y);
        min.z = Math.min(min.z, point.z);
        max.z = Math.max(max.z, point.z);
    }

    public void include(Vec2 point) {
        min.x = Math.min(min.x, point.x);
        max.x = Math.max(max.x, point.x);
        min.y = Math.min(min.y, point.y);
        max.y = Math.max(max.y, point.y);
    }

    public boolean overlaps3D(AABB other) {
        if (min.x >= other.max.x) return false;
        if (min.y >= other.max.y) return false;
        if (min.z >= other.max.z) return false;
        if (other.min.x >= max.x) return false;
        if (other.min.y >= max.y) return false;
        if (other.min.z >= max.z) return false;
        return true;
    }

    public boolean overlaps2D(AABB other) {
        if (min.x >= other.max.x) return false;
        if (min.y >= other.max.y) return false;
        if (other.min.x >= max.x) return false;
        if (other.min.y >= max.y) return false;
        return true;
    }

    @Override
    public boolean collidesWith(BoundingBox other) {
        return other.collidesWith(this);
    }

    @Override
    public boolean collidesWith(AABB other) {
        return overlaps3D(this);
    }

    @Override
    public boolean collidesWith(OBB other) {
        throw new UnimplementedCollisionException("AABB-OBB");
    }

    @Override
    public boolean collidesWith(ConvexHull other) {
        throw new UnimplementedCollisionException("AABB-ConvexHull");
    }

    @Override
    public boolean collidesWith(Union other) {
        for (BoundingBox bb : other.boundingBoxes) {
            if (bb.collidesWith(this)) return true;
        }
        return false;
    }
}