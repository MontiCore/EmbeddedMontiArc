/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bridge;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.components.BusUser;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;

public class Bridge extends BusUser {

    public final transient BridgeProperties properties;

    public Bridge(BridgeProperties properties) {
        super(properties);
        this.properties = properties;
    }

    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type){
            dispatchMessage((MessageSendEvent) event);
        } else if (type == MessageReceiveEvent.type) {
            MessageReceiveEvent msgRecvEvent = (MessageReceiveEvent) event;
            if (properties.delay.isZero()) {
                dispatchMessage(new MessageSendEvent(this, msgRecvEvent.getEventTime(), msgRecvEvent.getMessage()));
            } else {
                this.eesystem.simulator.addEvent(
                    new MessageSendEvent(
                        this, 
                        msgRecvEvent.getEventTime().plus(properties.delay),
                        msgRecvEvent.getMessage())
                    );
            }
        } else throw new UnexpectedEventException(this.toString(), event);
    }

    @Override
    protected void init() {

    }

}