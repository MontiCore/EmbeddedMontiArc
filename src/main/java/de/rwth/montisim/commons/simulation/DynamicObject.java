/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import de.rwth.montisim.commons.utils.Vec3;

/**
 * Represents a dynamic object in the simulation, does have positional and rotation velocities and a mass.
*/
public class DynamicObject extends StaticObject {
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