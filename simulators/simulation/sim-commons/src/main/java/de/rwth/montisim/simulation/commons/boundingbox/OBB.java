/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons.boundingbox;

import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;

@Typed("obb")
public class OBB implements BoundingBox {
    //public static final String TYPE_NAME = "obb";
    public Vec3 offset = new Vec3(); // Offset from center of mass
    public Vec3 half_extent = new Vec3(); // Half extent in every axis direction (positive and negative)
    public Mat3 axes = new Mat3(); // Normalized axes

    public Vec3 pos = new Vec3(); // World position
    public Mat3 world_space_half_axes = new Mat3();


    @Override
    public boolean collidesWith(BoundingBox other) {
        return other.collidesWith(this);
    }

    @Override
    public boolean collidesWith(AABB other) {
        throw new UnimplementedCollisionException("OBB-AABB");
    }

    @Override
    public boolean collidesWith(OBB other) {
        return CollisionTests.collides(this, other, new Vec3(), new Vec3());
    }

    @Override
    public boolean collidesWith(ConvexHull other) {
        return CollisionTests.collides(this, other);
    }

    @Override
    public boolean collidesWith(Union other) {
        for (BoundingBox bb : other.boundingBoxes) {
            if (bb.collidesWith(this)) return true;
        }
        return false;
    }

}