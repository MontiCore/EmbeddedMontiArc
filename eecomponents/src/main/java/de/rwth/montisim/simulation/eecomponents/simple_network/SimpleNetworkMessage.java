/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public class SimpleNetworkMessage extends Message {
    IPV6Address sender; // Is set by the Gateway component on send
    IPV6Address target; // Set to IPV6Address.BROADCAST_ADDR for local broadcast.

    public SimpleNetworkMessage(IPV6Address target, MessageInformation info, Object message, int msgLen) {
        super(info, message, msgLen);
        this.target = target;
    }

    public SimpleNetworkMessage(IPV6Address target, MessageInformation info, Object message) {
        super(info, message);
        this.target = target;
    }

    protected SimpleNetworkMessage() {
    }

    public static Object newMessage(IPV6Address target, Object payload) {
        return new Object[]{target.getString(), payload};
    }
}
