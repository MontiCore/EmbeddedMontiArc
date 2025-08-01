/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons;

import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.Typed;

/**
 * Represents a dynamic object in the simulation, does have positional and rotation velocities and a mass.
 */
@Typed("dynamic")
public class DynamicObject extends StaticObject {
    //public static final String TYPE = "dynamic";
    public Vec3 velocity;
    public Vec3 angularVelocity;
    public double mass;

    public DynamicObject(String type) {
        super(type);
        this.velocity = new Vec3();
        this.angularVelocity = new Vec3();
        this.mass = 0;
    }

    public String toString() {
        return "{name: " + name + ", type: " + type + ", mass: " + mass + ", pos: " + pos + ", vel: " + velocity + "}";
    }

}