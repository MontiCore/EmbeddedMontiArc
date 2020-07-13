/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import java.lang.reflect.InvocationTargetException;

import de.rwth.montisim.commons.simulation.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.components.EEComponent;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public class Sensor extends EEComponent implements Updatable {
    // Separate Sensor Logic & EEComponent to reuse sensor logic inside the actuator
    final SensorLogic logic;

    public Sensor(SensorProperties properties, PhysicalValue watchedValue, Updater updater) {
        super(properties);
        updater.addUpdatable(this);
        logic = new SensorLogic(properties, watchedValue);
    }

    @Override
    protected void init() throws EEMessageTypeException {
        logic.init(this);
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