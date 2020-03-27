/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.util.Optional;

import de.rwth.montisim.commons.boundingbox.BoundingBox;
import de.rwth.montisim.commons.utils.Mat3;
import de.rwth.montisim.commons.utils.Vec3;

/**
 * Data for a static object of the simulation world (cannot dynamically move, and has "infinite" mass).
 * Has a name and type for outside identification purposes (mainly debug).
 * Contains the position and rotation of the object.
 * The rotation is expressed as a rotation matrix: Multiplying a position in the object space gives its rotated world space.
 * Can also contain a BoundingBox. Specific implementations may vary (AABB, Sphere, OBB, ...)
 */
public class StaticObject {
    public String name;
    public String type;
    public Vec3 pos;
    public Mat3 rotation;
    public Optional<BoundingBox> bbox;

    public StaticObject(String type) {
        this.name = "unknown";
        this.type = type;
        this.pos = new Vec3();
        this.rotation = Mat3.unit();
        this.bbox = Optional.empty();
    }

    public String toString() {
        return "{name: " + name + ", type: " + type + ", pos: " + pos + "}";
    }
}