/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.Optional;

import de.rwth.montisim.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.commons.physicalvalue.PhysicalValueDouble;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;

@Typed(ActuatorProperties.TYPE)
public class ActuatorProperties extends BusUserProperties {
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

    protected ActuatorProperties() {
    }

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

    @Override
    public EEEventProcessor build(ComponentBuildContext context) {
        // TODO physical value name resolve as "config error"
        PhysicalValue val = context.physicalValues.getPhysicalValue(physical_value_name);
        if (!(val instanceof PhysicalValueDouble)) throw new IllegalArgumentException("Actuators can only work on Double type Physical values");
        return new Actuator(this, (PhysicalValueDouble) val, context.componentUpdater);
    }
}