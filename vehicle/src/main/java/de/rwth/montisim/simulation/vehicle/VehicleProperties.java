/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.Objects;
import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.Coordinates;
import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMissingComponentException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.environment.osmmap.OsmMap;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;
import de.rwth.montisim.simulation.vehicle.task.TaskProperties;

/**
 * General properties of the Vehicle. All lengths are given in Meters. All
 * masses in kg. All angles in degrees.
 */
public class VehicleProperties {
    // Partially taken from https://en.wikipedia.org/wiki/BMW_M4

    @JsonEntry("name")
    public String vehicleName = "UnnamedCar";

    public BodyProperties body;
    public WheelProperties wheels;

    public PowerTrainProperties powertrain;

    public PhysicsProperties physics;

    public final Vector<EEComponentProperties> components;

    public Optional<Vec2> start_pos = Optional.empty();
    public Optional<Coordinates> start_coords = Optional.empty();
    public Optional<Long> start_osm_node = Optional.empty();
    public double start_orientation = 0.0;

    public TaskProperties task;

    public VehicleProperties() {
        body = new BodyProperties();
        wheels = new WheelProperties();
        powertrain = new ElectricalPTProperties();
        physics = new RigidbodyPhysicsProperties();
        components = new Vector<>();
        powertrain.addDefaultActuators(this);
        task = new TaskProperties();
    }

    /**
     * Specifies the general dimensions of the Vehicle as well as its mass.
     */
    public static class BodyProperties /* implements JsonSerializable */ {
        public double mass = 1642;

        public double length = 4.971;
        public double width = 1.87;
        public double height = 1.383;

        public double center_of_gravity_height = 1.383 * 0.25; // Height of the CoG of the car with maximum suspension
        // extension.
    }

    /**
     * Specifies the position, size & properties of the wheels.
     */
    public static class WheelProperties /* implements JsonSerializable */ {
        public double diameter = (0.255 * 0.4) * 2 + (18 * Geometry.INCH_TO_METERS); // in meters: Tire width *
        // Aspect Ratio * 2 + (Rim
        // Diam * Inch)
        public double width = 0.255; // in meters

        // For direction wheels
        public double front_wheel_axis_offset = 1; // Distance between the wheel center (track) and the rotation point
        // of the wheels. (positive is inwards)
        public double max_turning_angle = 30; // In degrees

        // Wheel position
        public double front_track_width = 62.2 * Geometry.INCH_TO_METERS; // (Track: center of wheel) -> distance
        // between the 2 wheel's tracks
        public double back_track_dist = 63.1 * Geometry.INCH_TO_METERS;
        public double wheelbase = 2.812; // Distance between the front & back axes
        public double wheelbase_offset = 0; // Offset between the center of the wheelbase and the center of mass of the
        // vehicle (positive -> center of mass more to the front of the wheelbase).

    }

    public VehicleProperties setName(String name) {
        vehicleName = name;
        return this;
    }

    public Optional<EEComponentProperties> getComponentProperties(String componentName) {
        return components.stream().filter(x -> x.name.equals(componentName)).findFirst();
    }

    public void addComponent(EEComponentProperties properties) {
        // Override any existing component properties with the same name
        for (int i = 0; i < components.size(); ++i) {
            if (components.elementAt(i).name.equals(properties.name)) {
                components.set(i, properties);
                return;
            }
        }
        components.add(properties);
    }

    /**
     * Only adds the given properties to the config if it is not already present
     */
    public void addDefaultComponent(EEComponentProperties properties) {
        for (EEComponentProperties p : components) {
            if (p.name.equals(properties.name))
                return;
        }
        components.add(properties);
    }

    // public static class BuildContext {
    // public final MessageTypeManager mtManager;
    // public final Pathfinding pathfinding;
    // public final World world; // Can be null unless a PathGoal tries to resolve
    // lat/lon
    // public final OsmMap map; // Can be null unless a PathGoal tries to resolve
    // OSM-ids
    // public final Instant startTime;

    // public BuildContext(Pathfinding pathfinding, MessageTypeManager mtManager,
    // World world, OsmMap map, Instant startTime) {
    // this.pathfinding = pathfinding;
    // this.mtManager = mtManager;
    // this.world = world;
    // this.map = map;
    // this.startTime = startTime;
    // }
    // }

    public Vehicle build(BuildContext context)
            throws EEMessageTypeException, EESetupException, EEMissingComponentException {
        Vehicle target = new Vehicle(this);

        // Create EESimulator
        DiscreteEventSimulator sim = context.getObject(DiscreteEventSimulator.CONTEXT_KEY);
        target.eesystem = new EESystem(sim);

        // Create PowerTrain
        target.powerTrain = powertrain.build();

        // Create PhysicsModel
        target.physicsModel = physics.build(target.powerTrain, target.properties);
        target.physicalObject = target.physicsModel.getPhysicalObject();

        target.registerPhysicalValues();

        target.physicalObject.name = vehicleName;

        BuildContext compContext = new BuildContext(context);
        compContext.addObject(target.physicalValues);
        compContext.addObject(target.updater, EESystem.COMPONENT_UPDATER_CONTEXT_KEY);
        compContext.addObject(target.destroyer, EESystem.COMPONENT_DESTROYER_CONTEXT_KEY);
        compContext.addObject(target.popper, EESystem.COMPONENT_POPPER_CONTEXT_KEY);
        compContext.addObject(target.eesystem.getMsgPrioComp());
        compContext.addObject(target);

        // Add EEComponents
        for (EEComponentProperties properties : components) {
            properties.build(target.eesystem, compContext);
        }

        // Connect components
        for (EEComponent comp : target.eesystem.componentTable) {
            for (String other : comp.properties.connected_to) {
                comp.connectToComponent(other);
            }
        }

        // Resolve PowerTrain actuators
        target.powerTrain.gasActuator = target.eesystem.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        target.powerTrain.brakingActuator = target.eesystem.getActuator(PowerTrainProperties.BRAKING_ACTUATOR_NAME).get();
        target.powerTrain.steeringActuator = target.eesystem.getActuator(PowerTrainProperties.STEERING_ACTUATOR_NAME).get();

        target.eesystem.finalizeSetup();

        Objects.requireNonNull(task, "Task of Vehicle not specified");
        Optional<OsmMap> map = context.getOptionalObject(OsmMap.CONTEXT_KEY);
        Optional<World> world = context.getOptionalObject(World.CONTEXT_KEY);
        target.task = task.build(target, map, world);

        // Position Vehicle
        Vec2 pos = new Vec2();
        int c = 0;
        if (start_coords.isPresent()) {
            ++c;
            if (!world.isPresent())
                throw new IllegalArgumentException("World missing from BuildContext for vehicle start position");
            world.get().converter.get().coordsToMeters(start_coords.get(), pos);
        }
        if (start_pos.isPresent()) {
            ++c;
            pos.set(start_pos.get());
        }
        if (start_osm_node.isPresent()) {
            ++c;
            if (!map.isPresent())
                throw new IllegalArgumentException("Osm Map missing from BuildContext for vehicle start position");
            if (!world.isPresent())
                throw new IllegalArgumentException("World missing from BuildContext for vehicle start position");
            world.get().converter.get().coordsToMeters(map.get().getNode(start_osm_node.get()).coords, pos);
        }
        if (c == 0)
            throw new IllegalArgumentException("No start position/coordinates/osm-node specified for vehicle: " + vehicleName);
        if (c > 1)
            throw new IllegalArgumentException("Multiple start position/coordinates/osm-node specified for vehicle: " + vehicleName);

        target.physicsModel.setGroundPosition(
                new Vec3(pos.x, pos.y, 0),
                new Vec2(Math.cos(start_orientation * Geometry.DEG_TO_RAD), Math.sin(start_orientation * Geometry.DEG_TO_RAD))
        );

        return target;
    }
}