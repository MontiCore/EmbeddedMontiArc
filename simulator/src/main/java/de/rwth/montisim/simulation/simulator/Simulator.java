/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Duration;
import java.util.Vector;

import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.simulation.eecomponents.autopilots.JavaAutopilotProperties;
import de.rwth.montisim.simulation.eecomponents.autopilots.TestAutopilotProperties;
import de.rwth.montisim.simulation.eecomponents.navigation.NavigationProperties;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.DefaultVehicleConfig;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

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
        obj.state = new SimulatorState();
        obj.registerComponents(this);
    }

    public void removeSimulationObject(SimulationObject obj) {
        SimulatorState state = (SimulatorState) obj.state;

        if (state.staticId >= 0)
            staticObjects.set(state.staticId, null);
        if (state.dynamicId >= 0)
            dynamicObjects.set(state.dynamicId, null);
        if (state.updatableId >= 0)
            updatables.set(state.updatableId, null);
        if (state.destroyableId >= 0)
            destroyables.set(state.destroyableId, null);
        if (state.taskRunnerId >= 0)
            taskRunners.set(state.taskRunnerId, null);

        state.reset();
    }

    public VehicleBuilder getVehicleBuilder(VehicleProperties config) {
        return VehicleBuilder.fromConfig(mtManager, pathfinding, config);
    }

    public VehicleBuilder getDefaultVehicleBuilder() {
        return VehicleBuilder.fromConfig(mtManager, pathfinding, new DefaultVehicleConfig().properties);
    }

    @Override
    public void update(TimeUpdate newTime) {
        // TODO: OBSERVERS
        updatables.forEach(x -> {
            if (x != null)
                x.update(newTime);
        });
        simulatedTime = simulatedTime.plus(newTime.deltaTime);
        timeout = simulatedTime.compareTo(config.max_duration) > 0;
    }

    public boolean finished() {
        if (timeout) {
            return true;
        }
        for (TaskRunner r : taskRunners) {
            if (r == null)
                continue;
            if (r.status() != TaskStatus.SUCCEEDED)
                return false;
        }
        return true;
    }

    public boolean allTasksSucceeded() {
        int count = 0;
        for (TaskRunner r : taskRunners) {
            if (r == null)
                continue;
            if (r.status() != TaskStatus.SUCCEEDED)
                return false;
            count++;
        }
        if (timeout)
            return count == 0;
        return true;
    }

    private static class SimulatorState implements ISimulatorState {
        int staticId;
        int dynamicId;
        int updatableId;
        int destroyableId;
        int taskRunnerId;

        SimulatorState() {
            reset();
        }

        void reset() {
            staticId = -1;
            dynamicId = -1;
            updatableId = -1;
            destroyableId = -1;
            taskRunnerId = -1;
        }
    }

    /*
     * The flowing functions should be called by the SimulationObject being
     * registered through 'addSimulationObject(obj) -> obj.registerComponent()'.
     */
    @Override
    public void registerStaticObject(SimulationObject obj, StaticObject staticObject) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = staticObjects.size();
        staticObjects.add(staticObject);
    }

    @Override
    public void registerDynamicObject(SimulationObject obj, DynamicObject dynObject) {
        SimulatorState state = (SimulatorState) obj.state;
        state.dynamicId = dynamicObjects.size();
        dynamicObjects.add(dynObject);
    }

    @Override
    public void registerUpdatable(SimulationObject obj, Updatable updatable) {
        SimulatorState state = (SimulatorState) obj.state;
        state.updatableId = updatables.size();
        updatables.add(updatable);
    }

    @Override
    public void registerDestroyable(SimulationObject obj, Destroyable destroyable) {
        SimulatorState state = (SimulatorState) obj.state;
        state.destroyableId = destroyables.size();
        destroyables.add(destroyable);
    }

    @Override
    public void registerTaskRunner(SimulationObject obj, TaskRunner runner) {
        SimulatorState state = (SimulatorState) obj.state;
        state.updatableId = taskRunners.size();
        taskRunners.add(runner);
    }

    static {
        try {
            Json.registerType(NavigationProperties.class);
            Json.registerType(JavaAutopilotProperties.class);
            Json.registerType(TestAutopilotProperties.class);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            e.printStackTrace();
        }
    }
}