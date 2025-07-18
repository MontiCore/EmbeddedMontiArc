/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network.events;

import java.time.Instant;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.EventTarget;
import de.rwth.montisim.simulation.eecomponents.simple_network.SimpleNetworkMessage;

public class SimpleNetworkRecvEvent extends DiscreteEvent {
    public static final int type = registerType(SimpleNetworkRecvEvent.class);

    public SimpleNetworkMessage msg;

    public SimpleNetworkRecvEvent(EventTarget target, Instant time, SimpleNetworkMessage msg) {
        super(target, time);
        this.msg = msg;
    }

    @Override
    public int getType() {
        return type;
    }

}
