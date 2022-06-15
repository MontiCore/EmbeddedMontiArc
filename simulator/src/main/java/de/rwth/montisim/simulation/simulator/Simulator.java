/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.simulator;

import java.time.Duration;
import java.util.*;
import java.util.stream.Stream;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.simulation.*;
import de.rwth.montisim.simulation.commons.*;
import de.rwth.montisim.commons.utils.*;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.SerializationException;
import de.rwth.montisim.simulation.eecomponents.simple_network.SimulatorModule;
import de.rwth.montisim.simulation.eecomponents.vehicleconfigs.DefaultVehicleConfig;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Building;
import de.rwth.montisim.simulation.simulator.communication.DefaultPreprocessor;
import de.rwth.montisim.simulation.simulator.communication.DefaultPreprocessorProperties;
import de.rwth.montisim.simulation.simulator.communication.Preprocessor;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleBuilder;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

public class Simulator implements ISimulator, Updatable {
    public final SimulationConfig config;
    final World world;
    final Pathfinding pathfinding;
    final public BuildContext buildContext;
    public final DiscreteEventSimulator eventSimulator;
    public final Preprocessor preprocessor;
    final List<SimulatorModule> modules = new ArrayList<>();
    final HashMap<String, SimulatorModule> moduleByName = new HashMap<>();

    Duration simulatedTime = Duration.ZERO;

    private List<StaticObject> staticObjects = new ArrayList<>();
    private int staticObjectsCount = 0;
    private List<DynamicObject> dynamicObjects = new ArrayList<>();
    private int dynamicObjectsCount = 0;

    private List<Updatable> updatables = new ArrayList<>();
    private int updatablesCount = 0;
    private List<Destroyable> destroyables = new ArrayList<>();
    private int destroyablesCount = 0;

    private List<TaskRunner> taskRunners = new ArrayList<>();
    private int taskRunnersCount = 0;

    private HashMap<String, Vehicle> vehiclesByName = new HashMap<>();
    private List<Vehicle> vehicles = new ArrayList<>();
    private int vehiclesCount = 0;

    public HashMap<Triplet<String, String, Duration>, Duration> currentCollisions = new HashMap<>();

    public LinkedHashMap<Triplet<String, String, Duration>, Duration> collisionHistory = new LinkedHashMap<>();

    final CollisionDetection collisionDetection;

    boolean timeout = false;
    private boolean sim_initialized = false;

    // OsmMap can be null
    public Simulator(SimulationConfig config, World world, Pathfinding pathfinding, OsmMap map) {
        this.config = config;
        this.world = world;
        this.pathfinding = pathfinding;
        this.buildContext = new BuildContext();
        this.eventSimulator = new DiscreteEventSimulator(config.start_time);
        buildContext.addObject(pathfinding, Pathfinding.CONTEXT_KEY);
        buildContext.addObject(world);
        buildContext.addObject(map);
        buildContext.addObject(eventSimulator);

        // Build preprocessor
        if (!config.preprocessor.isPresent()) {
            config.preprocessor = Optional.of(new DefaultPreprocessorProperties());
        }
        preprocessor = config.preprocessor.get().build();

        for (Building b : world.buildings) {
            addSimulationObject(b);
        }

        collisionDetection = new CollisionDetection(vehicles, staticObjects);
        collisionDetection.loadStaticGrid();
    }

    public void addSimulationObject(SimulationObject obj) {
        obj.state = new SimulatorState();
        obj.registerComponents(this);
        if (obj instanceof Vehicle) {
            Vehicle v = (Vehicle) obj;
            if (vehiclesByName.containsKey(v.properties.vehicleName))
                throw new IllegalArgumentException("Error on adding Vehicle '" + v.properties.vehicleName + "' to the simulation: a vehicle with this name is already registered.");
            vehiclesByName.put(v.properties.vehicleName, v);
            SimulatorState state = (SimulatorState) obj.state;
            state.staticId = addObject(vehicles, vehiclesCount, v);
            ++vehiclesCount;
        }
    }

    public Vehicle getVehicle(String name) {
        return vehiclesByName.get(name);
    }

    public Stream<Vehicle> getVehicles() {
        return vehicles.stream().filter(x -> x != null);
    }

    /**
     * Deserializes and removes the vehicle from the simulation.
     */
    public String popVehicle(Vehicle v) throws SerializationException {
        v.pop();
        String state = v.stateToJson();
        removeSimulationObject(v);
        return state;
    }

    public void removeSimulationObject(SimulationObject obj) {
        SimulatorState state = (SimulatorState) obj.state;

        if (state.staticId >= 0) {
            staticObjects.set(state.staticId, null);
            --staticObjectsCount;
        }
        if (state.dynamicId >= 0) {
            dynamicObjects.set(state.dynamicId, null);
            --dynamicObjectsCount;
        }
        if (state.updatableId >= 0) {
            updatables.set(state.updatableId, null);
            --updatablesCount;
        }
        if (state.destroyableId >= 0) {
            destroyables.set(state.destroyableId, null);
            --destroyablesCount;
        }
        if (state.taskRunnerId >= 0) {
            taskRunners.set(state.taskRunnerId, null);
            --taskRunnersCount;
        }
        if (state.vehicleId >= 0) {
            vehicles.set(state.vehicleId, null);
            --vehiclesCount;
        }

        state.reset();
        if (obj instanceof Vehicle) {
            Vehicle v = (Vehicle) obj;
            vehiclesByName.remove(v.properties.vehicleName);
        }
    }

    public VehicleBuilder getVehicleBuilder(VehicleProperties config) {
        return VehicleBuilder.fromConfig(buildContext, config);
    }

    public VehicleBuilder getDefaultVehicleBuilder() {
        return VehicleBuilder.fromConfig(buildContext, new DefaultVehicleConfig().properties);
    }

    // Idea: "Once the simulation is set-up, compute the initial state (for "time=0")"
    private void initSimulation() {
        collisionDetection.checkCollisions();
    }

    // Idea: "Compute the new state of the simulation for the given new time"
    @Override
    public void update(TimeUpdate newTime) {
        if (!sim_initialized) {
            sim_initialized = true;
            initSimulation();
        }

        // TODO: OBSERVERS
        for (Updatable u : updatables) {
            if (u != null)
                u.update(newTime); // Includes vehicle physics
        }
        eventSimulator.update(newTime);
        collisionDetection.checkCollisions();

        simulatedTime = simulatedTime.plus(newTime.deltaTime);
        timeout = simulatedTime.compareTo(config.max_duration) > 0;
    }

    /**
     * @return SUCCEEDED if all tasks succeeded, FAILED if timeout and RUNNING else.
     */
    public TaskStatus status() {
        // TODO handle failures in tasks (early stop parameter?)
        // -> use 'any running ?' logic
        if (timeout) {
            return TaskStatus.FAILED_TIMEOUT;
        }
        ArrayList<Triplet<String, String, Duration>> col = getCollisions();
        if (config.collision_mode.equals("LOG_COLLISIONS")) {
            logNewCollisions(col);
        }
        if (!col.isEmpty()) {
            if (config.collision_mode.equals("FAIL_ON_COLLISIONS")) {
                return TaskStatus.FAILED_COLLISION;
            }

        }
        return allTasksSucceeded() ? TaskStatus.SUCCEEDED : TaskStatus.RUNNING;
    }

    private ArrayList<Triplet<String, String, Duration>> getCollisions() {
        ArrayList<Triplet<String, String, Duration>> res = new ArrayList<Triplet<String, String, Duration>>();
        for (Vehicle v : vehicles) {
            for (StaticObject o : v.staticCollisions) {
                res.add(new Triplet(v.properties.vehicleName, o.name, simulatedTime));
            }

            for (Vehicle v1 : v.vehicleCollisions) {
                res.add(new Triplet(v.properties.vehicleName, v1.properties.vehicleName, simulatedTime));
            }
        }

        return res;
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
            if (d != null)
                d.destroy();
        }
    }

    private static class SimulatorState implements ISimulatorState {
        int staticId;
        int dynamicId;
        int updatableId;
        int destroyableId;
        int taskRunnerId;
        int vehicleId;

        SimulatorState() {
            reset();
        }

        void reset() {
            staticId = -1;
            dynamicId = -1;
            updatableId = -1;
            destroyableId = -1;
            taskRunnerId = -1;
            vehicleId = -1;
        }
    }

    /*
     * The flowing functions should be called by the SimulationObject being
     * registered through 'addSimulationObject(obj) -> obj.registerComponent()'.
     */
    @Override
    public void registerStaticObject(SimulationObject obj, StaticObject staticObject) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = addObject(staticObjects, staticObjectsCount, staticObject);
        ++staticObjectsCount;
    }

    @Override
    public void registerDynamicObject(SimulationObject obj, DynamicObject dynObject) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = addObject(dynamicObjects, dynamicObjectsCount, dynObject);
        ++dynamicObjectsCount;
    }

    @Override
    public void registerUpdatable(SimulationObject obj, Updatable updatable) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = addObject(updatables, updatablesCount, updatable);
        ++updatablesCount;
    }

    @Override
    public void registerDestroyable(SimulationObject obj, Destroyable destroyable) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = addObject(destroyables, destroyablesCount, destroyable);
        ++destroyablesCount;
    }

    @Override
    public void registerTaskRunner(SimulationObject obj, TaskRunner runner) {
        SimulatorState state = (SimulatorState) obj.state;
        state.staticId = addObject(taskRunners, taskRunnersCount, runner);
        ++taskRunnersCount;
    }

    static private <T> int addObject(List<T> l, int count, T object) {
        int arraySize = l.size();
        int slot;
        if (arraySize > count) {
            slot = l.size() - 1;
            while (l.get(slot) != null) --slot;
            l.set(slot, object);
        } else {
            slot = arraySize;
            l.add(object);
        }
        return slot;
    }

    //log finished collision and calculate new collisions
    private void logNewCollisions(ArrayList<Triplet<String, String, Duration>> col) {
        List<Triplet<String, String, Duration>> newCollisions = new ArrayList(col);
        List<Triplet<String, String, Duration>> toRemove = new ArrayList();
        for (Triplet<String, String, Duration> active : currentCollisions.keySet()) {
            boolean found = false;
            for (Triplet<String, String, Duration> cur : col) {
                if (cur.getAt(0).equals(active.getAt(0)) && cur.getAt(1).equals(active.getAt(1))) {
                    currentCollisions.replace(active, currentCollisions.get(active).plus(config.tick_duration));
                    newCollisions.remove(cur);
                    found = true;
                    break;
                }
            }
            if (!found) {
                collisionHistory.put(active, currentCollisions.get(active));
                toRemove.add(active);
            }
        }
        for (Triplet<String, String, Duration> newCol : newCollisions) {
            currentCollisions.put(newCol, Duration.ZERO);
        }
        for (Triplet<String, String, Duration> item : toRemove) {
            currentCollisions.remove(item);
        }

    }

}