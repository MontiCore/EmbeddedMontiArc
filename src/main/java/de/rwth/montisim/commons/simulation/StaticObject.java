/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import java.util.Optional;

import de.rwth.montisim.commons.boundingbox.BoundingBox;
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
    public transient String desc;
    public Vec3 pos;
    public Mat3 rotation;
    public Optional<BoundingBox> bbox;

    public StaticObject(String desc) {
        this.name = "unknown";
        this.desc = desc;
        this.pos = new Vec3();
        this.rotation = Mat3.unit();
        this.bbox = Optional.empty();
    }

    public String toString() {
        return "{name: " + name + ", desc: " + desc + ", pos: " + pos + "}";
    }

}