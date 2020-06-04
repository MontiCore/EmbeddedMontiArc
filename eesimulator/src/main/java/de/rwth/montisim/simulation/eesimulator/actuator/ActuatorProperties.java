/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.Optional;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;

public class ActuatorProperties extends EEComponentProperties {
    public ActuatorProperties() {
        super(EEComponentType.ACTUATOR);
    }

    public ActuatorProperties(double minValue, double maxValue, double changeRate) {
        super(EEComponentType.ACTUATOR);
        this.maxValue = maxValue;
        this.minValue = minValue;
        this.changeRate = changeRate;
    }
    
    public ActuatorProperties setName(String name) {
        this.name = name;
        return this;
    }

    public ActuatorProperties setPhysicalValueName(String physicalValueName) {
        this.physicalValueName = physicalValueName;
        return this;
    }

    public ActuatorProperties setRange(double minValue, double maxValue) {
        this.minValue = minValue;
        this.maxValue = maxValue;
        return this;
    }

    public ActuatorProperties setChangeRate(double changeRate) {
        this.changeRate = changeRate;
        return this;
    }
    
    public ActuatorProperties setSensorProperties(SensorProperties properties) {
        this.sensorProperties = Optional.of(properties);
        return this;
    }


    public String physicalValueName;

    public double minValue;
    public double maxValue; 
    public double changeRate;
    public Optional<SensorProperties> sensorProperties;
}