/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.sensor;

import de.rwth.montisim.simulation.commons.physicalvalue.PhysicalValue;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.simulation.Updater;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;

public class Sensor extends EEComponent implements Updatable {
    // Separate Sensor Logic & EEComponent to reuse sensor logic inside the actuator
    final SensorLogic logic;

    public Sensor(SensorProperties properties, EESystem eesystem, PhysicalValue watchedValue, Updater updater)
            throws EEMessageTypeException {
        super(properties, eesystem);
        updater.addUpdatable(this);
        logic = new SensorLogic(this, properties, watchedValue);
    }

    @Override
    public void update(TimeUpdate newTime) {
        logic.update(newTime);
    }

    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
    }

}