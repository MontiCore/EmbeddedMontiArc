/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.util.HashMap;
import java.util.Optional;

import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.commons.utils.json.Json;
import de.rwth.montisim.commons.utils.json.JsonTraverser;
import de.rwth.montisim.commons.utils.json.JsonTraverser.Entry;
import de.rwth.montisim.commons.utils.json.JsonTraverser.ObjectIterable;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.actuator.*;
import de.rwth.montisim.simulation.eesimulator.bridge.*;
import de.rwth.montisim.simulation.eesimulator.bus.can.CAN;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANProperties;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.components.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.sensor.*;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestCompProperties;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;
import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysicsProperties;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.*;
import de.rwth.montisim.simulation.vehicle.powertrain.fuel.*;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class VehicleBuilder {

    public static VehicleBuilder fromConfig(MessageTypeManager mtManager, Pathfinding pathfinding,
            VehicleProperties config) {
        VehicleBuilder builder = new VehicleBuilder(mtManager, pathfinding);
        builder.config = Optional.of(config);
        return builder;
    }

    public static VehicleBuilder fromJsonConfig(MessageTypeManager mtManager, Pathfinding pathfinding,
            File configFile) {
        VehicleBuilder builder = new VehicleBuilder(mtManager, pathfinding);
        builder.file = Optional.of(configFile);
        builder.fromJson = true;
        builder.isState = false;
        return builder;
    }

    public static VehicleBuilder fromJsonConfig(MessageTypeManager mtManager, Pathfinding pathfinding,
            String configData) {
        VehicleBuilder builder = new VehicleBuilder(mtManager, pathfinding);
        builder.dataString = Optional.of(configData);
        builder.fromJson = true;
        builder.isState = false;
        return builder;
    }

    public static VehicleBuilder fromJsonState(MessageTypeManager mtManager, Pathfinding pathfinding, File configFile) {
        VehicleBuilder builder = new VehicleBuilder(mtManager, pathfinding);
        builder.file = Optional.of(configFile);
        builder.fromJson = true;
        builder.isState = true;
        return builder;
    }

    public static VehicleBuilder fromJsonState(MessageTypeManager mtManager, Pathfinding pathfinding,
            String configData) {
        VehicleBuilder builder = new VehicleBuilder(mtManager, pathfinding);
        builder.dataString = Optional.of(configData);
        builder.fromJson = true;
        builder.isState = true;
        return builder;
    }

    final MessageTypeManager mtManager;
    final Pathfinding pathfinding;
    final Vehicle target;
    boolean fromJson = false;
    boolean isState = false;
    Optional<VehicleProperties> config = Optional.empty();
    Optional<File> file = Optional.empty();
    Optional<String> dataString = Optional.empty();
    Optional<String> vehicleName = Optional.empty();

    // public VehicleBuilder(MessageTypeManager mtManager, Pathfinding pathfinding)
    // {
    // this.mtManager = mtManager;
    // this.pathfinding = pathfinding;
    // // Load default setup
    // target = new Vehicle();
    // this.config = new TestVehicleConfig();
    // }

    private VehicleBuilder(MessageTypeManager mtManager, Pathfinding pathfinding) {
        this.mtManager = mtManager;
        this.pathfinding = pathfinding;
        this.target = new Vehicle();
    }

    // public VehicleBuilder setAutopilot() {
    // // TODO
    // return this;
    // }

    // public VehicleBuilder addBus() {
    // // TODO
    // return this;
    // }

    // public VehicleBuilder addComponent() {
    // // TODO
    // return this;
    // }

    public VehicleBuilder setName(String name) {
        vehicleName = Optional.of(name);
        return this;
    }

    public Vehicle build() throws EESetupException, EEMessageTypeException, IOException, ParsingException,
            InstantiationException, IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException {
        if (fromJson) {
            JsonTraverser j = new JsonTraverser();
            if (file.isPresent()) {
                j.init(file.get());
            } else {
                j.init(dataString.get());
            }

            if (isState) {
                ObjectIterable it = j.streamObject();

                if (!it.iterator().hasNext())
                    j.expected(Vehicle.K_CONFIG);
                Entry e = it.iterator().next();
                if (!e.key.equals(Vehicle.K_CONFIG))
                    j.expected(Vehicle.K_CONFIG);
                config = Optional.of(Json.instantiateFromJson(j, VehicleProperties.class, null));
                buildFromConfig();

                if (!it.iterator().hasNext())
                    j.expected(Vehicle.K_STATE);
                e = it.iterator().next();
                if (!e.key.equals(Vehicle.K_STATE))
                    j.expected(Vehicle.K_STATE);
                Json.fromJson(j, target, null);
            } else {
                config = Optional.of(Json.instantiateFromJson(j, VehicleProperties.class, null));
                buildFromConfig();
            }
        } else {
            buildFromConfig();
        }
        return target;
    }

    private void buildFromConfig() throws EESetupException, EEMessageTypeException {
        VehicleProperties conf = config.get();
        // Create EESimulator
        target.eesimulator = new EESimulator(mtManager);

        target.properties = conf;

        // Create PowerTrain
        switch (conf.powertrain.getPowerTrainType()) {
            case ELECTRICAL:
                target.powerTrain = new ElectricalPowerTrain((ElectricalPTProperties) conf.powertrain);
                break;
            case FUEL_BASED:
                target.powerTrain = new FuelPowerTrain((FuelPTProperties) conf.powertrain);
                break;
            default:
                break;
        }

        target.powerTrain.registerPhysicalValues(target);

        // Create PhysicsModel
        switch (conf.physics.physicsType) {
            case MODELICA:
                // TODO
                throw new IllegalArgumentException("Missing Modelica implementation");
            case RIGIDBODY:
                target.physicsModel = new RigidbodyPhysics((RigidbodyPhysicsProperties) conf.physics, target.powerTrain,
                        target.properties);
                break;
            default:
                break;
        }

        target.physicalObject = target.physicsModel.getPhysicalObject();

        if (vehicleName.isPresent())
            target.physicalObject.name = vehicleName.get();
        else
            target.physicalObject.name = conf.vehicleName;

        BuildContext context = new BuildContext(target, pathfinding);
        // Add EEComponents
        for (EEComponentProperties properties : conf.components) {
            ComponentBuilder builder = componentBuilders.get(properties.getType());
            if (builder == null)
                throw new IllegalArgumentException(
                        "No ComponentBuilder registered for EEComponent type '" + properties.getType() + "'.");
            EEEventProcessor res = builder.build(properties, context);
            // switch(properties.componentType){
            // case ACTUATOR:
            // res = new Actuator(
            // (ActuatorProperties)properties,
            // target.getPhysicalValue(((ActuatorProperties)properties).physical_value_name),
            // target.updater
            // );
            // break;
            // case BRIDGE:
            // res = new Bridge((BridgeProperties)properties);
            // break;
            // case BUS:
            // res = Bus.buildBus((BusProperties)properties,
            // target.eesimulator.getMsgPrioComp());
            // break;
            // case COMPUTER:
            // res =
            // ComputerComponentBuilder.buildComputerComponent((ComputerComponentProperties)properties);
            // break;
            // case FUNCTION_BLOCK:
            // break;
            // case SENSOR:
            // res = new Sensor(
            // (SensorProperties)properties,
            // target.getPhysicalValue(((SensorProperties)properties).physical_value_name),
            // target.updater
            // );
            // break;
            // case SERVICE:
            // res =
            // ServiceComponentBuilder.buildServiceComponent((ServiceComponentProperties)properties,
            // pathfinding, target);
            // break;
            // case TEST_COMPONENT:
            // res = new TestEEComponent(properties.name);
            // break;
            // default:
            // break;
            // }
            res.attachTo(target.eesimulator);
        }

        ComponentManager cm = target.eesimulator.getComponentManager();

        // Connect to buses
        cm.componentTable.stream().filter(x -> x.getComponentType() != EEComponentType.BUS).forEach(x -> {
            BusUser bc = (BusUser) x;
            for (String busName : bc.properties.buses) {
                bc.connectToBus(busName);
            }
        });

        // Resolve PowerTrain actuators
        target.powerTrain.gasActuator = cm.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        target.powerTrain.brakingActuator = cm.getActuator(PowerTrainProperties.BRAKING_ACTUATOR_NAME).get();
        target.powerTrain.steeringActuator = cm.getActuator(PowerTrainProperties.STEERING_ACTUATOR_NAME).get();

        target.eesimulator.finalizeSetup();
    }

    public static class BuildContext {
        public final Vehicle vehicle;
        public final Pathfinding pathfinding;

        public BuildContext(Vehicle vehicle, Pathfinding pathfinding) {
            this.vehicle = vehicle;
            this.pathfinding = pathfinding;
        }
    }

    public static interface ComponentBuilder {
        EEEventProcessor build(EEComponentProperties properties, BuildContext context);
    }

    private static final HashMap<String, ComponentBuilder> componentBuilders = new HashMap<>();

    public static void registerComponentBuilder(String componentType, ComponentBuilder builder) {
        componentBuilders.put(componentType, builder);
    }

    static {
        registerComponentBuilder(ActuatorProperties.TYPE,
                (properties, context) -> new Actuator((ActuatorProperties) properties,
                        context.vehicle.getPhysicalValue(((ActuatorProperties) properties).physical_value_name),
                        context.vehicle.updater));
        registerComponentBuilder(SensorProperties.TYPE,
                (properties, context) -> new Sensor((SensorProperties) properties,
                        context.vehicle.getPhysicalValue(((SensorProperties) properties).physical_value_name),
                        context.vehicle.updater));
        registerComponentBuilder(BridgeProperties.TYPE,
                (properties, context) -> new Bridge((BridgeProperties) properties));
        registerComponentBuilder(TestCompProperties.TYPE,
                (properties, context) -> new TestEEComponent(properties.name));
        registerComponentBuilder(CANProperties.TYPE, (properties, context) -> new CAN((CANProperties) properties,
                context.vehicle.eesimulator.getMsgPrioComp()));
        registerComponentBuilder(ConstantBusProperties.TYPE,
                (properties, context) -> new ConstantBus((ConstantBusProperties) properties));

        try {
            Json.registerType(ElectricalPTProperties.class);
            Json.registerType(FuelPTProperties.class);
        } catch (IllegalArgumentException | IllegalAccessException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }
}