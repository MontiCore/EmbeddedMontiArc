/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.actuator;

import java.util.Optional;

import de.rwth.montisim.simulation.commons.physicalvalue.*;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.JsonEntry;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.sensor.SensorProperties;

@Typed(ActuatorProperties.TYPE)
public class ActuatorProperties extends EEComponentProperties {
    public static final String TYPE = "actuator";

    public String physical_value_name;

    public double change_rate = Double.POSITIVE_INFINITY;
    @JsonEntry("sensor")
    public Optional<SensorProperties> sensorProperties;

    public ActuatorProperties(double changeRate) {
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
    public EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException {
        // TODO physical value name resolve as "config error"
        PhysicalValueRegistry values = context.getObject(PhysicalValueRegistry.CONTEXT_KEY);
        PhysicalValue val = values.getPhysicalValue(physical_value_name);
        if (!(val instanceof PhysicalValueDouble))
            throw new IllegalArgumentException("Actuators can only work on Double type Physical values");
        Updater updater = context.getObject(EESystem.COMPONENT_UPDATER_CONTEXT_KEY);
        return new Actuator(this, eesystem, (PhysicalValueDouble) val, updater);
    }
}