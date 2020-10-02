/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.Optional;

import de.rwth.montisim.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonWriter;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;
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

    TaskStatus carStatus = TaskStatus.RUNNING;
    Optional<Vec2> target = Optional.empty();
    Vec2 a = new Vec2();
    Task task;

    protected Vehicle(VehicleProperties properties) {
        this.properties = properties;
    }

    // int c = 0;

    @Override
    public void update(TimeUpdate newTime) {
        // Update sensors & actuators WITH OLD TIME -> Gen new msgs & update actuation
        // values -> "Sync" between physics & EE
        updater.applyUpdate(newTime);
        // Update Physics & EESim (can be async)
        eesystem.simulator.update(newTime);
        physicsModel.update(newTime);

        task.update(this);
        carStatus = task.status();

        // TODO this is temp
//        double DIST_TO_TARGET = 5;
//        if (carStatus == TaskStatus.RUNNING) {
//            if (target.isPresent()) {
//                a.set(physicalObject.pos);
//                double dist = a.distance(target.get());
//                // if (c %100 == 0){
//                // System.out.println("Dist to target: "+dist);
//                // }
//                // c++;
//                if (dist < DIST_TO_TARGET) {
//                    carStatus = TaskStatus.SUCCEEDED;
//                }
//            }
//        }
    }

    // TODO this is temp
    public void addTarget(Vec2 t) {
        carStatus = TaskStatus.RUNNING;
        target = Optional.of(t);
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
        // TODO Auto-generated method stub

    }

    @Override
    public TaskStatus status() {
        // TODO this is temp
        return carStatus;
    }

    protected void addPhysicalValues() {
        physicalValues.addPhysicalValue(new TrueCompass(physicalObject));
        physicalValues.addPhysicalValue(new TrueVelocity(physicalObject));
        physicalValues.addPhysicalValue(new TruePosition(physicalObject));
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

    public void setTask(Task task) {
        this.task = task;
    }
}