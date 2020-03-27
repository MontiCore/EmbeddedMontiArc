/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class Vehicle implements Updatable {
    EESimulator eesimulator;
    VehicleProperties properties;
    PowerTrain power_train;
    PhysicsModel physics_model;
    DynamicObject physical_object;

    public Vehicle(VehicleSetup setup){
        eesimulator = new EESimulator();
        properties = setup.getProperties();
        physical_object = new DynamicObject("vehicle");
        power_train = setup.getPowerTrain(eesimulator);
        physics_model = setup.getPhysicsModel(physical_object, power_train);
    }

    @Override
    public void update(TimeUpdate newTime) {
        //TODO update EESimulator, PowerTrain, PhysicsModel, Sensors, ...
    }
}