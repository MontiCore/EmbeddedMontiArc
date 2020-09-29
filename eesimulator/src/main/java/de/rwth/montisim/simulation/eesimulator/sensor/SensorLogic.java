/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Instant;

import de.rwth.montisim.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SensorLogic implements Updatable {
    final transient PhysicalValue watchedValue;
    final transient SensorProperties properties;

    Instant nextUpdate = Instant.EPOCH; // Time after which the sensor can send its next value

    transient MessageInformation msgInfo;
    transient EEComponent component;

    public SensorLogic(SensorProperties properties, PhysicalValue watchedValue) {
        this.watchedValue = watchedValue;
        this.properties = properties;
    }

    public void init(EEComponent component) throws EEMessageTypeException {
        this.msgInfo = component.addOutput(watchedValue.name, watchedValue.getType());
        this.component = component;
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (newTime.oldTime.isAfter(nextUpdate)) {
            this.nextUpdate = newTime.oldTime.plus(properties.update_interval);
            Object readValue = readValue();
            if (!properties.send_only_changed || watchedValue.hasChanged()) {
                component.sendMessage(newTime.oldTime.plus(properties.read_time), msgInfo, readValue);
            }
        }
    }

    // Can be overwritten to add noise, bias, ...
    public Object readValue() {
        return watchedValue.get();
    }
}