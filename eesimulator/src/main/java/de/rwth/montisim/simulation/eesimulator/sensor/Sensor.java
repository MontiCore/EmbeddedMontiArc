/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.time.Duration;

import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;

public class Sensor extends EEComponent implements Updatable {
    // Separate Sensor Logic & EEComponent to reuse sensor logic inside the actuator
    final SensorLogic logic;

    public Sensor(EESimulator simulator, String name, int priority, PhysicalValue watchedValue, Duration updateInterval, Duration readTime, boolean sendOnlyChanged, Updater updater) {
        super(simulator, name, priority);
        logic = new SensorLogic(watchedValue, updateInterval, readTime, sendOnlyChanged, this);
        updater.addUpdatable(this);
    }

    @Override
    public void update(TimeUpdate newTime) {
        logic.update(newTime);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
    }

    @Override
    public EEComponentType getComponentType() {
        return EEComponentType.SENSOR;
    }
}