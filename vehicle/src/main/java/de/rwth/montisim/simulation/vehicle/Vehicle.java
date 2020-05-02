/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.simulation.Destroyable;
import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.ISimulator;
import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.vehicleproperties.VehicleProperties;

public class Vehicle extends Updater implements SimulationObject, Updatable, Destroyable {
    public EESimulator eesimulator;
    public VehicleProperties properties;
    public PowerTrain power_train;
    public PhysicsModel physics_model;
    public DynamicObject physical_object;

    public Vehicle(VehicleSetup setup) throws EESetupException {
        eesimulator = new EESimulator();
        properties = setup.getProperties();
        power_train = setup.getPowerTrain(eesimulator);
        physics_model = setup.getPhysicsModel(eesimulator, this);
        physical_object = physics_model.getPhysicalObject();
        setup.addComponents(eesimulator, this);
        eesimulator.finalizeSetup();
    }

    @Override
    public void update(TimeUpdate newTime) {
        // Update sensors & actuators WITH OLD TIME -> Gen new msgs & update actuation
        // values -> "Sync" between physics & EE
        applyUpdate(newTime);
        // Update Physics & EESim (can be async)
        eesimulator.update(newTime);
        physics_model.update(newTime);
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerUpdatable(this);
        simulator.registerDynamicObject(physical_object);
        simulator.registerDestroyable(this);
    }

    @Override
    public void destroy() {
        // TODO Auto-generated method stub

    }

}