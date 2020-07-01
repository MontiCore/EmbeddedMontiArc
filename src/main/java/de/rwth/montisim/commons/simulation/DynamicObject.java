/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

import de.rwth.montisim.commons.utils.JsonTraverser;
import de.rwth.montisim.commons.utils.JsonWriter;
import de.rwth.montisim.commons.utils.StringRef;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.JsonTraverser.Entry;

/**
 * Represents a dynamic object in the simulation, does have positional and rotation velocities and a mass.
*/
public class DynamicObject extends StaticObject {
    public static final String TYPE = "dynamic";
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
        return "{name: " + name + ", type: " + desc + ", mass: " + mass + ", pos: " + pos + ", vel: " + velocity + "}";
    }



    
    public static final String K_VEL = "vel";
    public static final String K_ANGULAR_VEL = "ang_vel";
    public static final String K_MASS = "mass";

    @Override
    public void toJson(JsonWriter j) {
        j.startObject();
        j.write(JsonTraverser.K_TYPE, TYPE);
        writeStatic(j);
        writeDynamic(j);
        j.endObject();
    }

    protected void writeDynamic(JsonWriter j){
        j.writeKey(K_VEL);
        velocity.toJson(j);
        j.writeKey(K_ANGULAR_VEL);
        angularVelocity.toJson(j);
        j.write(K_MASS, mass);
    }

    @Override
    public void fromJson(JsonTraverser j) {
        for (Entry e : j.expectStructureType(TYPE)){
            if (!readStatic(j, e.key) && !readDynamic(j, e.key)) j.unexpected(e);
        }
    }

    protected boolean readDynamic(JsonTraverser j, StringRef key){
        if (key.equals(K_VEL)) {
            velocity.fromJson(j);
        } else if (key.equals(K_ANGULAR_VEL)) {
            angularVelocity.fromJson(j);
        } else if (key.equals(K_MASS)) {
            mass = j.getDouble();
        } else return false;
        return true;
    }
}