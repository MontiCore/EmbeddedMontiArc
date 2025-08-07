/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.constant;

import java.time.Duration;

import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bus.*;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties.BusType;
import de.rwth.montisim.simulation.eesimulator.events.*;

/**
 * Model for instant transmission of BusMessageEvent.
 */
public class ConstantBus extends Bus {
    public final transient ConstantBusProperties properties;

    public ConstantBus(ConstantBusProperties properties, EESystem eesystem) {
        super(properties, eesystem);
        this.properties = properties;
    }

    @Override
    protected void receive(MessageReceiveEvent event) {
        switch (properties.mode) {
            case INSTANT:
                // Directly dispatch the message
                dispatchMessage(new MessageSendEvent(null, event.getEventTime(), event.getMessage()));
                break;
            case CONSTANT_RATE:
                double time = event.getMessage().msgLen / properties.rate;
                Duration d = Time.durationFromSeconds(time);
                eesystem.simulator.addEvent(new MessageSendEvent(this, event.getEventTime().plus(d), event.getMessage()));
                break;
            case CONSTANT_TIME:
                eesystem.simulator.addEvent(
                        new MessageSendEvent(this, event.getEventTime().plus(properties.time), event.getMessage())
                );
                break;
        }
    }

    @Override
    protected void messageSent(MessageSendEvent event) {
        // Nothing to do, the message is already dispatched
    }

    @Override
    public BusType getBusType() {
        return BusType.CONSTANT_BUS;
    }

}
