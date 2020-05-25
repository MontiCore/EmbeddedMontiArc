/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle;

import java.util.HashMap;

import de.rwth.montisim.commons.simulation.Destroyable;
import de.rwth.montisim.commons.simulation.DynamicObject;
import de.rwth.montisim.commons.simulation.ISimulator;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.SimulationObject;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicalvalues.PhysicalValueBuilder;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueCompass;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TruePosition;
import de.rwth.montisim.simulation.vehicle.physicalvalues.TrueVelocity;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class Vehicle extends Updater implements SimulationObject, Updatable, Destroyable {
    // Register all the Physical Values statically HERE so that they are guaranteed to be loaded for the calls to 'getPhysicalValue()'
    static {
        PhysicalValueBuilder.registerPhysicalValueBuilder(TrueVelocity.VALUE_NAME, (Vehicle vehicle) -> new TrueVelocity(vehicle.physicalObject));
        PhysicalValueBuilder.registerPhysicalValueBuilder(TruePosition.VALUE_NAME, (Vehicle vehicle) -> new TruePosition(vehicle.physicalObject));
        PhysicalValueBuilder.registerPhysicalValueBuilder(TrueCompass.VALUE_NAME, (Vehicle vehicle) -> new TrueCompass(vehicle.physicalObject));
    }
    public EESimulator eesimulator;
    public VehicleProperties properties;
    public PowerTrain powerTrain;
    public PhysicsModel physicsModel;
    public DynamicObject physicalObject;

    HashMap<String, PhysicalValue> physicalValues = new HashMap<>();

    /** Should only be for RESOLVING the physical values, not repeated access. */
    public PhysicalValue getPhysicalValue(String name) {
        PhysicalValue res = physicalValues.get(name);
        if (res == null){
            res = PhysicalValueBuilder.buildPhysicalValue(name, this);
            physicalValues.put(name, res);
        }
        return res;
    }

    public void addPhysicalValue(PhysicalValue value){
        physicalValues.put(value.name, value);
    }

    protected Vehicle() {
    }

    @Override
    public void update(TimeUpdate newTime) {
        // Update sensors & actuators WITH OLD TIME -> Gen new msgs & update actuation
        // values -> "Sync" between physics & EE
        applyUpdate(newTime);
        // Update Physics & EESim (can be async)
        eesimulator.update(newTime);
        physicsModel.update(newTime);
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerUpdatable(this);
        simulator.registerDynamicObject(physicalObject);
        simulator.registerDestroyable(this);
    }

    @Override
    public void destroy() {
        // TODO Auto-generated method stub

    }

}