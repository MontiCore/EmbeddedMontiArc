/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.controller.commons;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Provides all information for a vertex data structure for a general graph. It
 * is widely used in this function block setting.
 */
@Deprecated
public class Vertex {

    public long id;
    public long osmId;
    public double maximumSteeringAngle;
    public Vec3 pos;
    public boolean intersectionNode = false;

    /// Takes ownership of the given position!
    public Vertex(long id, long osmID, Vec3 position, double maximumSteeringAngle) {
        this.id = id;
        this.osmId = osmID;
        this.pos = position;
        this.maximumSteeringAngle = maximumSteeringAngle;
    }

    public Vertex clone() {
        return new Vertex(id, osmId, this.pos.clone(), maximumSteeringAngle);
    }

    @Override
    public boolean equals(Object obj) {
        if (obj.getClass() != this.getClass()) {
            return false;
        }
        Vertex o = (Vertex) obj;
        if (o.maximumSteeringAngle != maximumSteeringAngle)
            return false;
        if (o.osmId != osmId)
            return false;
        if (o.id != id)
            return false;
        if (!o.pos.equals(pos))
            return false;
        return true;
    }
}
