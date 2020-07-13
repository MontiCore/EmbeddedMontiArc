/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.simulation.eesimulator.actuator.ActuatorProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

public abstract class PowerTrainProperties /*implements JsonSerializable*/ {
    public static final String GAS_VALUE_NAME = "gas";
    public static final String BRAKING_VALUE_NAME = "braking";
    public static final String STEERING_VALUE_NAME = "steering";
    
    public static final String GAS_ACTUATOR_NAME = "GasActuator";
    public static final String BRAKING_ACTUATOR_NAME = "BrakingActuator";
    public static final String STEERING_ACTUATOR_NAME = "SteeringActuator";
    
    public static enum PowerTrainType {
        ELECTRICAL,
        FUEL_BASED
    }
    
    public static enum TractionType {
        @JsonEntry("front")
        FRONT,
        @JsonEntry("rear")
        REAR,
        @JsonEntry("all")
        ALL // 4x4
    }
    public TractionType traction = TractionType.REAR;
    public TractionType braking = TractionType.REAR;

    // TODO check
    public double max_braking_force = 5000; // In Newtons


    
    public PowerTrainProperties(VehicleProperties config){
        // TODO read some default properties from "VehicleProperties" ()
        // Add default actuator description
        if (config == null) return;
        config.addDefaultComponent(
            new ActuatorProperties(-30, 30, 60)
            .setPhysicalValueName(STEERING_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
            .setName(STEERING_ACTUATOR_NAME)
            //.setInternal()
        );
        config.addDefaultComponent(
            new ActuatorProperties(0, 1, 10)
            .setPhysicalValueName(BRAKING_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
            .setName(BRAKING_ACTUATOR_NAME)
            //.setInternal()
        );
        config.addDefaultComponent(
            new ActuatorProperties(-0.5, 1, Double.POSITIVE_INFINITY)
            .setPhysicalValueName(GAS_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
            .setName(GAS_ACTUATOR_NAME)
            //.setInternal()
        );
        
    }

    public abstract PowerTrainType getPowerTrainType();
}