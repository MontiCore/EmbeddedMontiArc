/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Duration;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValueRegistry;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.EEComponent;

@Typed(SensorProperties.TYPE)
public class SensorProperties extends EEComponentProperties {
    public static final String TYPE = "sensor";

    public String physical_value_name;

    public Duration update_interval;
    public Duration read_time;
    public boolean send_only_changed;

    public SensorProperties(Duration updateInterval, Duration readTime, boolean sendOnlyChanged) {
        this.update_interval = updateInterval;
        this.read_time = readTime;
        this.send_only_changed = sendOnlyChanged;
    }

    public SensorProperties() {
        this.update_interval = Duration.ofMillis(100);
        this.read_time = Duration.ofMillis(10);
        this.send_only_changed = false;
    }


    public SensorProperties setPhysicalValueName(String physicalValueName) {
        this.physical_value_name = physicalValueName;
        return this;
    }

    public SensorProperties setUpdateInterval(Duration updateInterval) {
        this.update_interval = updateInterval;
        return this;
    }

    public SensorProperties setReadTime(Duration readTime) {
        this.read_time = readTime;
        return this;
    }

    public SensorProperties setUpdateRule(boolean sendOnlyChanged) {
        this.send_only_changed = sendOnlyChanged;
        return this;
    }

    @Override
    public EEComponentType getGeneralType() {
        return EEComponentType.SENSOR;
    }

    @Override
    public String getType() {
        return TYPE;
    }

    @Override
    public EEComponent build(EESystem eesystem, BuildContext context) throws EEMessageTypeException {
        PhysicalValueRegistry values = context.getObject(PhysicalValueRegistry.CONTEXT_KEY);
        PhysicalValue val = values.getPhysicalValue(physical_value_name);
        Updater updater = context.getObject(EESystem.COMPONENT_UPDATER_CONTEXT_KEY);
        return new Sensor(this, eesystem, val, updater);
    }
}