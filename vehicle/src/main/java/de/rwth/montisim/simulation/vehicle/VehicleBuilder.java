/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle;

import java.util.Vector;

import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.actuator.*;
import de.rwth.montisim.simulation.eesimulator.bridge.*;
import de.rwth.montisim.simulation.eesimulator.bus.*;
import de.rwth.montisim.simulation.eesimulator.components.*;
import de.rwth.montisim.simulation.eesimulator.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.sensor.*;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;
import de.rwth.montisim.simulation.environment.pathfinding.Pathfinding;
import de.rwth.montisim.simulation.vehicle.componentbuilders.*;
import de.rwth.montisim.simulation.vehicle.config.*;
import de.rwth.montisim.simulation.vehicle.physicsmodel.rigidbody.RigidbodyPhysics;
import de.rwth.montisim.simulation.vehicle.powertrain.electrical.*;
import de.rwth.montisim.simulation.vehicle.powertrain.fuel.*;
import de.rwth.montisim.simulation.vehicle.powertrain.PowerTrainProperties;

public class VehicleBuilder {
    final MessageTypeManager mtManager;
    final Pathfinding pathfinding;
    final Vehicle target;
    final VehicleConfig config;
    String vehicleName;

    // public VehicleBuilder(MessageTypeManager mtManager, Pathfinding pathfinding) {
    //     this.mtManager = mtManager;
    //     this.pathfinding = pathfinding;
    //     // Load default setup
    //     target = new Vehicle();
    //     this.config = new TestVehicleConfig();
    // }

    public VehicleBuilder(MessageTypeManager mtManager, Pathfinding pathfinding, VehicleConfig config) {
        this.mtManager = mtManager;
        this.pathfinding = pathfinding;
        target = new Vehicle();
        this.config = config;
    }

    // public VehicleBuilder setAutopilot() {
    //     // TODO
    //     return this;
    // }

    // public VehicleBuilder addBus() {
    //     // TODO
    //     return this;
    // }

    // public VehicleBuilder addComponent() {
    //     // TODO
    //     return this;
    // }

    public VehicleBuilder setName(String name) {
        config.vehicleName = name;
        return this;
    }

    public Vehicle build() throws EESetupException, EEMessageTypeException {
        // Create EESimulator
        target.eesimulator = new EESimulator(mtManager);

        // Get VehicleProperties
        target.properties = config.properties;
        
        // Create PowerTrain
        switch (config.powerTrainProperties.powerTrainType) {
        case ELECTRICAL:
            target.powerTrain = new ElectricalPowerTrain((ElectricalPTProperties)config.powerTrainProperties);
            break;
        case FUEL_BASED:
            target.powerTrain = new FuelPowerTrain((FuelPTProperties)config.powerTrainProperties);
            break;
        default:
            break;
        }

        target.powerTrain.registerPhysicalValues(target);

        // Create PhysicsModel
        switch (config.physicsProperties.physicsType) {
            case MODELICA:
                // TODO
                throw new IllegalArgumentException("Missing Modelica implementation");
            case RIGIDBODY:
            target.physicsModel = new RigidbodyPhysics(target.powerTrain, target.properties);
                break;
            default:
                break;
        }

        target.physicalObject = target.physicsModel.getPhysicalObject();
        target.physicalObject.name = config.vehicleName;

        Vector<EEComponentProperties> propertiesByComponentId = new Vector<>();
        propertiesByComponentId.setSize(config.eeConfig.components.size());
        // Add EEComponents
        // TODO
        for (EEComponentProperties properties : config.eeConfig.components){
            EEEventProcessor res = null;
            switch(properties.componentType){
                case ACTUATOR:
                    res = new Actuator(
                        (ActuatorProperties)properties, 
                        target.getPhysicalValue(((ActuatorProperties)properties).physicalValueName), 
                        target
                    );
					break;
                case BRIDGE:
                    res = new Bridge((BridgeProperties)properties);
					break;
                case BUS:
                    res = Bus.buildBus((BusProperties)properties, target.eesimulator.getMsgPrioComp());
					break;
                case COMPUTER:
                    res = ComputerComponentBuilder.buildComputerComponent((ComputerComponentProperties)properties);
					break;
				case FUNCTION_BLOCK:
					break;
                case SENSOR:
                    res = new Sensor(
                        (SensorProperties)properties, 
                        target.getPhysicalValue(((SensorProperties)properties).physicalValueName), 
                        target
                    );
					break;
                case SERVICE:
                    res = ServiceComponentBuilder.buildServiceComponent((ServiceComponentProperties)properties, pathfinding, target);
					break;
                case TEST_COMPONENT:
                    res = new TestEEComponent(properties.name);
					break;
				default:
					break;
            }
            res.attachTo(target.eesimulator);
            propertiesByComponentId.set(res.id, properties);
        }

        ComponentManager cm = target.eesimulator.getComponentManager();

        // Connect to buses
        cm.componentTable.stream().
            filter(x -> x.getComponentType() != EEComponentType.BUS).
            forEach(x -> {
                BusComponent bc = (BusComponent) x;
                EEComponentProperties properties = propertiesByComponentId.elementAt(x.id);
                for (String busName : properties.buses){
                    bc.connectToBus(busName);
                }
            });

        // Resolve PowerTrain actuators
        target.powerTrain.gasActuator = cm.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        target.powerTrain.brakingActuator = cm.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        target.powerTrain.steeringActuator = cm.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        
        target.eesimulator.finalizeSetup();
        return target;
    }
}