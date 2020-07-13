/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel;

public abstract class PhysicsProperties {
    public static enum PhysicsType {
        RIGIDBODY,
        MODELICA
    }
    public final transient PhysicsType physicsType;

    public PhysicsProperties(PhysicsType physicsType){
        this.physicsType = physicsType;
    }

}