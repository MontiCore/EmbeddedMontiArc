/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.physicsmodel;

import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public abstract class PhysicsProperties {
    public static enum PhysicsType {
        RIGIDBODY,
        MODELICA
    }

    public abstract PhysicsModel build(PowerTrain powerTrain, VehicleProperties properties);

    public abstract PhysicsType getPhysicsType();
}