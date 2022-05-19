/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties.BusType;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.events.*;

/**
 * Buses are used to transmit messages in the vehicle between EEComponents (and
 * through Bridges). Current implementations: ConstantBus, CAN & FlexRay.
 */
public abstract class Bus extends EEComponent {

    protected Bus(BusProperties properties, EESystem eesystem) {
        super(properties, eesystem);
    }

    /**
     * Accepts MessageSendEvent and MessageReceiveEvent events.
     * MessageSendEvent: When a component connected to the bus starts to (attempt to) transmit a message.
     * MessageReceiveEvent: When a message transmission is completed (message gets passed to its targets).
     * This Event is only created by the BUS itself.
     */
    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type) {
            MessageSendEvent msgRecvEvent = (MessageSendEvent) event;
            dispatchMessage(msgRecvEvent);
            messageSent(msgRecvEvent);
        } else if (type == MessageReceiveEvent.type) {
            receive((MessageReceiveEvent) event);

        } else
            throw new UnexpectedEventException(this.toString(), event);
    }

    protected abstract void messageSent(MessageSendEvent event);

    public abstract BusType getBusType();

    @Override
    public String toString() {
        return getBusType() + " bus \"" + properties.name + '"';
    }

}
