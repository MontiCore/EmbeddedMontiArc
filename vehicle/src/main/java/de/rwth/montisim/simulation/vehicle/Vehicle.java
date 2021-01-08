/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import de.rwth.montisim.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPowerTrain;
import de.rwth.montisim.simulation.vehicle.task.Task;

public class Vehicle extends SimulationObject implements Updatable, Destroyable, TaskRunner {
    public static final boolean SERIALIZE_FORMATTED = true;

    public final transient VehicleProperties properties;
    public EESystem eesystem;
    public PowerTrain powerTrain;
    public PhysicsModel physicsModel;
    public DynamicObject physicalObject;

    public transient final PhysicalValueRegistry physicalValues = new PhysicalValueRegistry();
    public transient final Updater updater = new Updater();
    public transient final Destroyer destroyer = new Destroyer();

    public Task task; // Task with no goals => always succeeds

    protected Vehicle(VehicleProperties properties) {
        this.properties = properties;
    }

    @Override
    public void update(TimeUpdate newTime) {
        // Update sensors & actuators WITH OLD TIME -> Gen new msgs & update actuation
        // values -> "Sync" between physics & EE
        updater.applyUpdate(newTime);
        // Update Physics & EESim (can be async)
        eesystem.simulator.update(newTime);
        physicsModel.update(newTime);

        task.update(this);
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerUpdatable(this, this);
        simulator.registerDynamicObject(this, physicalObject);
        simulator.registerDestroyable(this, this);
        simulator.registerTaskRunner(this, this);
    }

    @Override
    public void destroy() {
        destroyer.applyDestroy();
    }

    @Override
    public TaskStatus status() {
        return task.status();
    }

    protected void addPhysicalValues() {
        if ( powerTrain instanceof ElectricalPowerTrain  ) {
            physicalValues.addPhysicalValue(new BatteryLevel( ((ElectricalPowerTrain) powerTrain).battery));
        }
        physicalValues.addPhysicalValue(new TrueCompass(physicalObject));
        physicalValues.addPhysicalValue(new TrueVelocity(physicalObject));
        physicalValues.addPhysicalValue(new TruePosition(physicalObject));
        physicalValues.addPhysicalValue(new UpperSpeedLimit(physicalObject));
    }

    public static final String K_CONFIG = "config";
    public static final String K_STATE = "state";

    public String stateToJson() throws SerializationException {
        JsonWriter w = new JsonWriter(SERIALIZE_FORMATTED);
        w.startObject();
        w.writeKey(K_CONFIG);
        Json.toJson(w, properties, null);
        w.writeKey(K_STATE);
        Json.toJson(w, this, null);
        w.endObject();
        return w.getString();
    }

    /**
     * For tests
     *
     * @return
     */
    public PhysicalValueRegistry getPhysicalValues() {

        return physicalValues;
    }

}