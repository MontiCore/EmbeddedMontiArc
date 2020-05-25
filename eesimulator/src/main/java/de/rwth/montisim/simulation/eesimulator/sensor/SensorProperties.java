/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Duration;

import de.rwth.montisim.simulation.eesimulator.components.EEComponentProperties;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;

public class SensorProperties extends EEComponentProperties {
    public SensorProperties() {
        super(EEComponentType.SENSOR);
    }
    public SensorProperties(Duration updateInterval, Duration readTime, boolean sendOnlyChanged) {
        super(EEComponentType.SENSOR);
        this.updateInterval = updateInterval;
        this.readTime = readTime;
        this.sendOnlyChanged = sendOnlyChanged;
    }
    
    public SensorProperties setName(String name) {
        this.name = name;
        return this;
    }
    
    public SensorProperties setPhysicalValueName(String physicalValueName) {
        this.physicalValueName = physicalValueName;
        return this;
    }

    public SensorProperties setUpdateInterval(Duration updateInterval) {
        this.updateInterval = updateInterval;
        return this;
    }
    public SensorProperties setReadTime(Duration readTime) {
        this.readTime = readTime;
        return this;
    }
    public SensorProperties setUpdateRule(boolean sendOnlyChanged) {
        this.sendOnlyChanged = sendOnlyChanged;
        return this;
    }

    public String physicalValueName;

    public Duration updateInterval;
    public Duration readTime; 
    public boolean sendOnlyChanged;
}