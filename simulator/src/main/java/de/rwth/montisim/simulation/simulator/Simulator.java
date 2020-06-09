/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Duration;
import java.util.Vector;

import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.DefaultVehicleConfig;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.config.VehicleConfig;

public class Simulator implements ISimulator, Updatable {
    final SimulationConfig config;
    final World world;
    final Pathfinding pathfinding;
    final MessageTypeManager mtManager;

    Duration simulatedTime = Duration.ZERO;

    Vector<StaticObject> staticObjects = new Vector<>();
    Vector<DynamicObject> dynamicObjects = new Vector<>();

    Vector<Updatable> updatables = new Vector<>();
    Vector<Destroyable> destroyables = new Vector<>();

    Vector<TaskRunner> taskRunners = new Vector<>();

    boolean timeout = false;

    public Simulator(SimulationConfig config, World world, Pathfinding pathfinding, MessageTypeManager mtManager) {
        this.config = config;
        this.world = world;
        this.pathfinding = pathfinding;
        this.mtManager = mtManager;
        // TODO load static objects of the World
    }

    public void addSimulationObject(SimulationObject obj) {
        obj.registerComponents(this);
    }

    public VehicleBuilder getVehicleBuilder(VehicleConfig config) {
        return new VehicleBuilder(mtManager, pathfinding, config);
    }

    public VehicleBuilder getDefaultVehicleBuilder() {
        return new VehicleBuilder(mtManager, pathfinding, new DefaultVehicleConfig());
    }

    @Override
    public void update(TimeUpdate newTime) {
        // TODO: OBSERVERS
        updatables.forEach(x -> x.update(newTime));
        simulatedTime = simulatedTime.plus(newTime.deltaTime);
        timeout = simulatedTime.compareTo(config.maxSimulationDuration) > 0;
    }

    public boolean finished() {
        if (timeout) {
            return true;
        }
        for (TaskRunner r : taskRunners){
            if (r.status() != TaskStatus.SUCCEEDED) return false;
        }
        return true;
    }

    public boolean allTasksSucceeded() {
        for (TaskRunner r : taskRunners){
            if (r.status() != TaskStatus.SUCCEEDED) return false;
        }
        if (timeout) return taskRunners.size() == 0;
        return true;
    }

    @Override
    public void registerStaticObject(StaticObject staticObject) {
        staticObjects.add(staticObject);
    }

    @Override
    public void registerDynamicObject(DynamicObject dynObject) {
        dynamicObjects.add(dynObject);
    }

    @Override
    public void registerUpdatable(Updatable updatable) {
        updatables.add(updatable);
    }

    @Override
    public void registerDestroyable(Destroyable destroyable) {
        destroyables.add(destroyable);
    }

    @Override
    public void registerTaskRunner(TaskRunner runner) {
        taskRunners.add(runner);
    }

}