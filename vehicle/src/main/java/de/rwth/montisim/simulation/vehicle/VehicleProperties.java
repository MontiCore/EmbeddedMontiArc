/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.Optional;
import java.util.Vector;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.map.Pathfinding;
import de.rwth.montisim.commons.utils.Geometry;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.components.BusUser;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties.ComponentBuildContext;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.vehicle.physicsmodel.PhysicsProperties;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.ElectricalPTProperties;

/**
 * General properties of the Vehicle. All lengths are given in Meters. All
 * masses in kg. All angles in degrees.
 */
public class VehicleProperties /* implements JsonSerializable */ {
    // Partially taken from https://en.wikipedia.org/wiki/BMW_M4

    @JsonEntry("name")
    public String vehicleName = "UnnamedCar";

    public BodyProperties body;
    public WheelProperties wheels;

    public PowerTrainProperties powertrain;

    public PhysicsProperties physics;

    public final Vector<EEComponentProperties> components;

    public Vec2 start_coords = new Vec2();
    public Vec2 end_coords;

    public VehicleProperties(){
        body = new BodyProperties();
        wheels = new WheelProperties();
        powertrain = new ElectricalPTProperties();
        physics = new RigidbodyPhysicsProperties();
        components = new Vector<>();
        powertrain.addDefaultActuators(this);
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
        public double wheelbase_offset = 0; // Offset between the center of the wheelbase and the center of mass of the vehicle (positive -> center of mass more to the front of the wheelbase).

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

    public static class BuildContext {
        public final MessageTypeManager mtManager;
        public final Pathfinding pathfinding;

        public BuildContext(Pathfinding pathfinding, MessageTypeManager mtManager) {
            this.pathfinding = pathfinding;
            this.mtManager = mtManager;
        }
    }

    public Vehicle build(BuildContext context) throws EEMessageTypeException, EESetupException {
        Vehicle target = new Vehicle(this);

        // Create EESimulator
        target.eesystem = new EESystem(new DiscreteEventSimulator(), context.mtManager);

        // Create PowerTrain
        target.powerTrain = powertrain.build();
        target.powerTrain.registerPhysicalValues(target.physicalValues);

        // Create PhysicsModel
        target.physicsModel = physics.build(target.powerTrain, target.properties);
        target.physicalObject = target.physicsModel.getPhysicalObject();

        target.addPhysicalValues();

        target.physicalObject.name = vehicleName;

        ComponentBuildContext compContext = new ComponentBuildContext(
            target.physicalValues, 
            target.updater, 
            target.eesystem.getMsgPrioComp(), 
            context.pathfinding
            );
        // Add EEComponents
        for (EEComponentProperties properties : components) {
            EEEventProcessor res = properties.build(compContext);
            res.attachTo(target.eesystem);
        }

        ComponentManager cm = target.eesystem.getComponentManager();

        // Connect to buses
        cm.componentTable.stream().filter(x -> x.properties.getGeneralType() != EEComponentType.BUS).forEach(x -> {
            BusUser bc = (BusUser) x;
            for (String busName : bc.properties.buses) {
                bc.connectToBus(busName);
            }
        });

        // Resolve PowerTrain actuators
        target.powerTrain.gasActuator = cm.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        target.powerTrain.brakingActuator = cm.getActuator(PowerTrainProperties.BRAKING_ACTUATOR_NAME).get();
        target.powerTrain.steeringActuator = cm.getActuator(PowerTrainProperties.STEERING_ACTUATOR_NAME).get();

        target.eesystem.finalizeSetup();
        return target;
    }

}