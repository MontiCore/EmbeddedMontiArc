/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Instant;

import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SensorLogic implements Updatable {
    final transient PhysicalValue watchedValue;
    final transient SensorProperties properties;
    
    Instant nextUpdate = Instant.EPOCH; // Time after which the sensor can send its next value
    Object lastValue = null; // TODO serialize using type

    transient MessageInformation msgInfo;
    transient EEComponent component;

    public SensorLogic(SensorProperties properties, PhysicalValue watchedValue) {
        this.watchedValue = watchedValue;
        this.properties = properties;
    }

    public void init(EEComponent component) throws EEMessageTypeException {
        this.msgInfo = component.addOutput(watchedValue.name, watchedValue.type);
        this.component = component;
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (newTime.oldTime.isAfter(nextUpdate)){
            this.nextUpdate = newTime.oldTime.plus(properties.update_interval);
            Object readValue = readValue();
            if (!properties.send_only_changed || hasChanged(readValue)){
                lastValue = readValue;
                component.sendMessage(newTime.oldTime.plus(properties.read_time), msgInfo, readValue);
            }
        }
    }

    private boolean hasChanged(Object value) {
        // TODO
        // !UMath.equalsThreshold(readValue, lastValue, 0.00001f)
        return true;
    }

    // Can be overwritten to add noise, bias, ...
    public Object readValue(){
        return watchedValue.get();
    }
}