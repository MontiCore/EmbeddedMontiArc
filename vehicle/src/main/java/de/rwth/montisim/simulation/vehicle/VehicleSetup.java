/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;

public interface VehicleSetup {
    public VehicleProperties getProperties();
    public PowerTrain getPowerTrain(EESimulator eesimulator);
    public PhysicsModel getPhysicsModel(EESimulator eesimulator, Updater vehicle);
    public void addComponents(EESimulator eesimulator, Updater vehicle);
}