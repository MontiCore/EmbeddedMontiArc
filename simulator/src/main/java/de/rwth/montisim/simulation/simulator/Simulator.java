/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Duration;
import java.util.Collection;
import java.util.HashMap;
import java.util.Vector;

import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.simulation.eecomponents.autopilots.JavaAutopilotProperties;
import de.rwth.montisim.simulation.eecomponents.autopilots.TestAutopilotProperties;
import de.rwth.montisim.simulation.eesimulator.actuator.ActuatorProperties;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestCompProperties;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.simulator.vehicleconfigs.DefaultVehicleConfig;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties.BuildContext;
import de.rwth.montisim.simulation.vehicle.navigation.NavigationProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.fuel.FuelPTProperties;
import de.rwth.montisim.simulation.vehicle.task.metric.MetricGoalProperties;
import de.rwth.montisim.simulation.vehicle.task.path.PathGoalProperties;

public class Simulator implements ISimulator, Updatable {
    public final SimulationConfig config;
    final World world;
    final Pathfinding pathfinding;
    final MessageTypeManager mtManager;
    final public BuildContext buildContext;

    Duration simulatedTime = Duration.ZERO;

    Vector<StaticObject> staticObjects = new Vector<>();
    Vector<DynamicObject> dynamicObjects = new Vector<>();

    Vector<Updatable> updatables = new Vector<>();
    Vector<Destroyable> destroyables = new Vector<>();

    Vector<TaskRunner> taskRunners = new Vector<>();
    HashMap<String, Vehicle> vehicles = new HashMap<>();

    boolean timeout = false;

    // OsmMap can be null
    public Simulator(SimulationConfig config, World world, Pathfinding pathfinding, MessageTypeManager mtManager, OsmMap map) {
        this.config = config;
        this.world = world;
        this.pathfinding = pathfinding;
        this.mtManager = mtManager;
        this.buildContext = new BuildContext(pathfinding, mtManager, world, map, config.start_time);
        // TODO load static objects of the World
    }

    public void addSimulationObject(SimulationObject obj) {
        obj.state = new SimulatorState();
        obj.registerComponents(this);
        if (obj instanceof Vehicle) {
            Vehicle v = (Vehicle) obj;
            if (vehicles.containsKey(v.properties.vehicleName)) throw new IllegalArgumentException("Error on adding Vehicle '"+v.properties.vehicleName+"' to the simulation: a vehicle with this name is already registered.");
            vehicles.put(v.properties.vehicleName, v);
        }
    }

    public Vehicle getVehicle(String name) {
        return vehicles.get(name);
    }

    public Collection<Vehicle> getVehicles() {
        return vehicles.values();
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
        if (obj instanceof Vehicle) {
            Vehicle v = (Vehicle) obj;
            vehicles.remove(v.properties.vehicleName);
        }
    }

    public VehicleBuilder getVehicleBuilder(VehicleProperties config) {
        return VehicleBuilder.fromConfig(buildContext, config);
    }

    public VehicleBuilder getDefaultVehicleBuilder() {
        return VehicleBuilder.fromConfig(buildContext, new DefaultVehicleConfig().properties);
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

    /**
     * @return SUCCEEDED if all tasks succeeded, FAILED if timeout and RUNNING else.
     */
    public TaskStatus status() {
        if (timeout) {
            return TaskStatus.FAILED;
        }
        return allTasksSucceeded() ? TaskStatus.SUCCEEDED : TaskStatus.RUNNING;
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

    // Call to clean-up all object that require explicit clean-up (ex: hardware_emulator)
    public void destroy() {
        for (Destroyable d : destroyables) {
            d.destroy();
        }
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
        state.taskRunnerId = taskRunners.size();
        taskRunners.add(runner);
    }

    public static void registerJsonTypes() {
        Json.registerType(NavigationProperties.class);
        Json.registerType(JavaAutopilotProperties.class);
        Json.registerType(TestAutopilotProperties.class);
        Json.registerType(ElectricalPTProperties.class);
        Json.registerType(FuelPTProperties.class);
        Json.registerType(NavigationProperties.class);
        Json.registerType(JavaAutopilotProperties.class);
        Json.registerType(TestAutopilotProperties.class);
        Json.registerType(RigidbodyPhysicsProperties.class);
        Json.registerType(ActuatorProperties.class);
        Json.registerType(BridgeProperties.class);
        Json.registerType(CANProperties.class);
        Json.registerType(ConstantBusProperties.class);
        Json.registerType(SensorProperties.class);
        Json.registerType(TestCompProperties.class);
        Json.registerType(PathGoalProperties.class);
        Json.registerType(MetricGoalProperties.class);
    }

    public Vector<Updatable> getUpdatables() {
        return updatables;
    }
}