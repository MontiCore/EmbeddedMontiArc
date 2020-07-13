/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.Optional;

import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;

@Typed(ActuatorProperties.TYPE)
public class ActuatorProperties extends EEComponentProperties {
    public static final String TYPE = "actuator";

    public String physical_value_name;

    public double min;
    public double max;
    public double change_rate;
    @JsonEntry("sensor")
    public Optional<SensorProperties> sensorProperties;

    public ActuatorProperties(double minValue, double maxValue, double changeRate) {
        this.max = maxValue;
        this.min = minValue;
        this.change_rate = changeRate;
    }

    protected ActuatorProperties() {}

    // TODO
    // public ActuatorProperties setName(String name) {
    // this.name = name;
    // return this;
    // }

    public ActuatorProperties setPhysicalValueName(String physicalValueName) {
        this.physical_value_name = physicalValueName;
        return this;
    }

    public ActuatorProperties setRange(double minValue, double maxValue) {
        this.min = minValue;
        this.max = maxValue;
        return this;
    }

    public ActuatorProperties setChangeRate(double changeRate) {
        this.change_rate = changeRate;
        return this;
    }

    public ActuatorProperties setSensorProperties(SensorProperties properties) {
        this.sensorProperties = Optional.of(properties);
        return this;
    }



    
    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.ACTUATOR;
    }
    @Override
    public String getType() {
        return TYPE;
    }
}