/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.config;

import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public abstract class VehicleConfig {

    public String vehicleName = "UnnamedCar";

    public VehicleProperties properties;

    public PowerTrainProperties powerTrainProperties;

    public PhysicsProperties physicsProperties;

    public EEConfig eeConfig;
    
}