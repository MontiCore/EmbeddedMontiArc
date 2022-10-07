/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain;

import de.rwth.montisim.simulation.commons.physicalvalue.*;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;

public abstract class PowerTrain {
    public static final double MAX_STEERING_ANGLE = 30.0;

    transient public final PowerTrainProperties properties;
    transient public Motor motor;

    @JsonEntry(PowerTrainProperties.STEERING_VALUE_NAME)
    public final PhysicalValueDouble steeringValue; // In degrees
    @JsonEntry(PowerTrainProperties.BRAKING_VALUE_NAME)
    public final PhysicalValueDouble brakingValue;
    @JsonEntry(PowerTrainProperties.GAS_VALUE_NAME)
    public final PhysicalValueDouble gasValue;

    transient public Actuator steeringActuator; // In degrees
    transient public Actuator brakingActuator; // From 0 to 1 (no brake - full brake)
    transient public Actuator gasActuator; // From -0.5 to 1 (backwards - forward)

    public PowerTrain(PowerTrainProperties properties) {
        this.properties = properties;
        this.steeringValue = new PhysicalValueDouble(PowerTrainProperties.STEERING_VALUE_NAME, 0.0, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
        this.brakingValue = new PhysicalValueDouble(PowerTrainProperties.BRAKING_VALUE_NAME, 1.0, 0, 1);
        this.gasValue = new PhysicalValueDouble(PowerTrainProperties.GAS_VALUE_NAME, 0.0, -0.5, 1.0);
    }

    public void registerPhysicalValues(PhysicalValueRegistry physicalValues) {
        physicalValues.addPhysicalValue(steeringValue);
        physicalValues.addPhysicalValue(brakingValue);
        physicalValues.addPhysicalValue(gasValue);
    }

    public void resolveActuators(EESystem eeSystem) {
        gasActuator = eeSystem.getActuator(PowerTrainProperties.GAS_ACTUATOR_NAME).get();
        brakingActuator = eeSystem.getActuator(PowerTrainProperties.BRAKING_ACTUATOR_NAME).get();
        steeringActuator = eeSystem.getActuator(PowerTrainProperties.STEERING_ACTUATOR_NAME).get();
    }

    public abstract double getFuelPercentage();

    /**
     * @return The current ratio between the motor & the wheels (ratio = motor revolutions / wheel revolutions)
     */
    public abstract double getTransmissionRatio();
}