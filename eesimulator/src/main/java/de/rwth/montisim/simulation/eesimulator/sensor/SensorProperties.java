/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Duration;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.BusUserProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

@Typed(SensorProperties.TYPE)
public class SensorProperties extends BusUserProperties {
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

    protected SensorProperties() {}

    public SensorProperties setName(String name) {
        this.name = name;
        return this;
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
}