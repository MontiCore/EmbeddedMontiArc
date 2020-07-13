/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.vehicle.powertrain;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.simulation.eesimulator.actuator.Actuator;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public abstract class PowerTrain {

    transient public final PowerTrainProperties properties;
    transient public Motor motor;
    
    @JsonEntry("steering")
    public final PhysicalValue steeringValue; // In degrees
    @JsonEntry("braking")
    public final PhysicalValue brakingValue;
    @JsonEntry("gas")
    public final PhysicalValue gasValue;
    
    transient public Actuator steeringActuator; // In degrees
    transient public Actuator brakingActuator; // From 0 to 1 (no brake - full brake)
    transient public Actuator gasActuator; // From -0.5 to 1 (backwards - forward)

    public PowerTrain(PowerTrainProperties properties) {
        this.properties = properties;
        this.steeringValue = new PhysicalValue(PowerTrainProperties.STEERING_VALUE_NAME, DataType.DOUBLE, 0.0);
        this.brakingValue = new PhysicalValue(PowerTrainProperties.BRAKING_VALUE_NAME, DataType.DOUBLE, 0.0);
        this.gasValue = new PhysicalValue(PowerTrainProperties.GAS_VALUE_NAME, DataType.DOUBLE, 0.0);
    }

    public void registerPhysicalValues(Vehicle vehicle){
        vehicle.addPhysicalValue(steeringValue);
        vehicle.addPhysicalValue(brakingValue);
        vehicle.addPhysicalValue(gasValue);
    }

    public abstract double getFuelPercentage();
    /**
     * @return The current ratio between the motor & the wheels (ratio = motor revolutions / wheel revolutions)
     */
    public abstract double getTransmissionRatio();
}