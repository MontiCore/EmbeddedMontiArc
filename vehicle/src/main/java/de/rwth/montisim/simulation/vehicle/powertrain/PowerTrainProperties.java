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
    public TractionType braking = TractionType.FRONT;

    // TODO check
    public double max_braking_force = 5000; // In Newtons


    public void addDefaultActuators(VehicleProperties config) {
        config.addDefaultComponent(
                new ActuatorProperties(60)
                        .setPhysicalValueName(STEERING_VALUE_NAME)
                        .setSensorProperties(new SensorProperties(
                                Duration.ofMillis(100),
                                Duration.ofMillis(1),
                                false
                        ))
                        .setName(STEERING_ACTUATOR_NAME)
                        .connectTo("DefaultBus")
                //.setInternal()
        );
        config.addDefaultComponent(
                new ActuatorProperties(10)
                        .setPhysicalValueName(BRAKING_VALUE_NAME)
                        .setSensorProperties(new SensorProperties(
                                Duration.ofMillis(100),
                                Duration.ofMillis(1),
                                false
                        ))
                        .setName(BRAKING_ACTUATOR_NAME)
                        .connectTo("DefaultBus")
                //.setInternal()
        );
        config.addDefaultComponent(
                new ActuatorProperties(Double.POSITIVE_INFINITY)
                        .setPhysicalValueName(GAS_VALUE_NAME)
                        .setSensorProperties(new SensorProperties(
                                Duration.ofMillis(100),
                                Duration.ofMillis(1),
                                false
                        ))
                        .setName(GAS_ACTUATOR_NAME)
                        .connectTo("DefaultBus")
                //.setInternal()
        );
    }

    public abstract PowerTrain build();

    public abstract PowerTrainType getPowerTrainType();
}