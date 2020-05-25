/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SensorLogic implements Updatable {
    final PhysicalValue watchedValue;
    final SensorProperties properties;
    
    Instant nextUpdate = Instant.EPOCH; // Time after which the sensor can send its next value
    Object lastValue = null;

    MessageInformation msgInfo;
    EEComponent component;

    public SensorLogic(SensorProperties properties, PhysicalValue watchedValue) {
        this.watchedValue = watchedValue;
        this.properties = properties;
    }

    public void init(EEComponent component) throws EEMessageTypeException {
        this.msgInfo = component.addOutput(watchedValue.name, DataType.DOUBLE);
        this.component = component;
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (newTime.oldTime.isAfter(nextUpdate)){
            this.nextUpdate = newTime.oldTime.plus(properties.updateInterval);
            Object readValue = readValue();
            if (!properties.sendOnlyChanged || hasChanged(readValue)){
                lastValue = readValue;
                component.send(newTime.oldTime.plus(properties.readTime), new Message(msgInfo, readValue, component.id));
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