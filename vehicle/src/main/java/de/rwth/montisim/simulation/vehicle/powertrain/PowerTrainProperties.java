/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.vehicle.powertrain;

import java.time.Duration;

import de.rwth.montisim.simulation.eesimulator.actuator.ActuatorProperties;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;
import de.rwth.montisim.simulation.vehicle.config.EEConfig;

public abstract class PowerTrainProperties {
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
    public final PowerTrainType powerTrainType;
    
    public static enum TractionType {
        FRONT,
        REAR,
        ALL // 4x4
    }
    public TractionType tractionType = TractionType.REAR;
    public TractionType brakingType = TractionType.REAR;

    // TODO check
    public double maxBrakingForce = 5000; // In Newtons


    
    public PowerTrainProperties(PowerTrainType powerTrainType, EEConfig eeConfig){
        this.powerTrainType = powerTrainType;
        // Add default actuator description
        eeConfig.addComponent(
            new ActuatorProperties(-30, 30, 60)
            .setName(STEERING_ACTUATOR_NAME)
            .setPhysicalValueName(STEERING_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
        );
        eeConfig.addComponent(
            new ActuatorProperties(0, 1, 10)
            .setName(BRAKING_ACTUATOR_NAME)
            .setPhysicalValueName(BRAKING_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
        );
        eeConfig.addComponent(
            new ActuatorProperties(-0.5, 1, Double.POSITIVE_INFINITY)
            .setName(GAS_ACTUATOR_NAME)
            .setPhysicalValueName(GAS_VALUE_NAME)
            .setSensorProperties(new SensorProperties(
                Duration.ofMillis(100), 
                Duration.ofMillis(1), 
                false
            ))
        );
        
    }

}