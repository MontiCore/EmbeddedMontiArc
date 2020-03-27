/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.physicsmodel.masspoint.MasspointPhysics;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;

public abstract class VehicleSetup {
    public VehicleProperties getProperties() {
        return new VehicleProperties();
    }
    public PowerTrain getPowerTrain(EESimulator ee_vehicle){
        return new ElectricalPowerTrain(ee_vehicle);
    }
    public PhysicsModel getPhysicsModel(DynamicObject physical_data, PowerTrain power_train){
        return new MasspointPhysics(physical_data, power_train);
    }

}