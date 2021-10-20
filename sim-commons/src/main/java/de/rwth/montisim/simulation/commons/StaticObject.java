/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons;

import java.util.Optional;

import de.rwth.montisim.simulation.commons.boundingbox.AABB;
import de.rwth.montisim.simulation.commons.boundingbox.BoundingBox;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;

/**
 * Data for a static object of the simulation world (cannot dynamically move,
 * and has "infinite" mass) with a bounding box.
 */
@Typed("static")
public class StaticObject {
    //public static final String TYPE = "static";
    public transient String name;
    public transient String type;
    public Vec3 pos;
    public Mat3 rotation;
    public Optional<BoundingBox> bbox;
    public Optional<AABB> worldSpaceAABB;

    public StaticObject(String type) {
        this.name = "unknown";
        this.type = type;
        this.pos = new Vec3();
        this.rotation = Mat3.unit();
        this.bbox = Optional.empty();
        this.worldSpaceAABB = Optional.empty();
    }

    public String toString() {
        return "{name: " + name + ", type: " + type + ", pos: " + pos + "}";
    }

}