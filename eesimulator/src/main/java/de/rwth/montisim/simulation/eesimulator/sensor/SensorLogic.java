/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Duration;
import java.time.Instant;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.utils.UMath;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SensorLogic implements Updatable {
    final PhysicalValue watchedValue;
    final Duration updateInterval;
    final Duration readTime;
    Instant nextUpdate = Instant.EPOCH; // Time after which the sensor can send its next value
    double lastValue = Double.NaN;
    final boolean sendOnlyChanged;

    final MessageInformation msgInfo;
    final EEComponent component;

    public SensorLogic(PhysicalValue watchedValue, Duration updateInterval, Duration readTime, boolean sendOnlyChanged, EEComponent component) {
        this.watchedValue = watchedValue;
        this.updateInterval = updateInterval;
        this.readTime = readTime;
        this.sendOnlyChanged = sendOnlyChanged;
        this.msgInfo = component.addOutput(watchedValue.name, DataType.DOUBLE);
        this.component = component;
    }

    @Override
    public void update(TimeUpdate newTime) {
        if (newTime.oldTime.isAfter(nextUpdate)){
            this.nextUpdate = newTime.oldTime.plus(updateInterval);
            double readValue = readValue();
            if (!sendOnlyChanged || !UMath.equalsThreshold(readValue, lastValue, 0.00001f)){
                lastValue = readValue;
                component.send(newTime.oldTime.plus(readTime), new Message(msgInfo, readValue, component.id));
            }
        }
    }

    // Can be overwritten to add noise, bias, ...
    public double readValue(){
        return watchedValue.get();
    }
}