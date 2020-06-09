/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.HashMap;
import java.util.Optional;

import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.vehicle.physicalvalues.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsModel;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrain;

public class Vehicle extends Updater implements SimulationObject, Updatable, Destroyable, TaskRunner {
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

    TaskStatus carStatus = TaskStatus.SUCCEEDED;
    Optional<Vec2> target = Optional.empty();
    Vec2 a = new Vec2();

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

    int c = 0;
    @Override
    public void update(TimeUpdate newTime) {
        // Update sensors & actuators WITH OLD TIME -> Gen new msgs & update actuation
        // values -> "Sync" between physics & EE
        applyUpdate(newTime);
        // Update Physics & EESim (can be async)
        eesimulator.update(newTime);
        physicsModel.update(newTime);

        // TODO this is temp
        double DIST_TO_TARGET = 5;
        if (carStatus == TaskStatus.RUNNING) {
            if (target.isPresent()){
                a.set(physicalObject.pos);
                double dist = a.distance(target.get());
                // if (c %100 == 0){
                //     System.out.println("Dist to target: "+dist);
                // }
                // c++;
                if (dist < DIST_TO_TARGET){
                    carStatus = TaskStatus.SUCCEEDED;
                }
            }
        }
    }

    // TODO this is temp
    public void addTarget(Vec2 t) {
        carStatus = TaskStatus.RUNNING;
        target = Optional.of(t);
    }

    @Override
    public void registerComponents(ISimulator simulator) {
        simulator.registerUpdatable(this);
        simulator.registerDynamicObject(physicalObject);
        simulator.registerDestroyable(this);
        simulator.registerTaskRunner(this);
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

}