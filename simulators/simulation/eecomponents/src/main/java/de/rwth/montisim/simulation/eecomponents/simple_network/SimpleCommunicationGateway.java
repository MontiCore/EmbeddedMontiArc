/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.util.HashSet;
import java.util.Set;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.SimplePacketType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eecomponents.simple_network.events.SimpleNetworkRecvEvent;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.message.PortTagUser;
import de.rwth.montisim.simulation.vehicle.Vehicle;

public class SimpleCommunicationGateway extends EEComponent implements PortTagUser {
    public static final String NETWORK_TAG = "network";
    public static final Set<String> tagSet = new HashSet<>();

    static {
        tagSet.add(NETWORK_TAG);
    }

    transient public final SCGProperties properties;

    transient final SimpleNetwork network;
    public transient final IPV6Address address;
    transient final Vehicle vehicle;

    public SimpleCommunicationGateway(SCGProperties properties, EESystem eeSystem, SimpleNetwork network, Vehicle vehicle) {
        super(properties, eeSystem);
        this.properties = properties;
        this.network = network;
        this.vehicle = vehicle;
        this.address = network.registerNode(vehicle.properties.vehicleName + "_COMM_GATEWAY", vehicle.physicalObject, this);
    }


    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type) {
            dispatchMessage((MessageSendEvent) event);
        } else if (type == MessageReceiveEvent.type) {
            receive((MessageReceiveEvent) event);
        } else if (type == SimpleNetworkRecvEvent.type) {
            receive((SimpleNetworkRecvEvent) event);
        } else throw new UnexpectedEventException(this.toString(), event);
    }


    // Receiving a message from a vehicle component -> Send to the Network
    @Override
    protected void receive(MessageReceiveEvent msgRecvEvent) {
        Message msg = msgRecvEvent.getMessage();
        Object packet[] = (Object[]) msg.message;
        SimpleNetworkMessage netMsg = new SimpleNetworkMessage(new IPV6Address((String) packet[0]), msg.msgInfo, packet[1], msg.msgLen);
        netMsg.sender = address;
        eesystem.simulator.addEvent(new SimpleNetworkRecvEvent(network, msgRecvEvent.getEventTime().plus(properties.processing_time), netMsg));
    }

    // Receiving a message from the Network
    private void receive(SimpleNetworkRecvEvent event) {
        MessageInformation msgInfo = getMsgInfo(event.msg.msgInfo.name);
        if (msgInfo == null) return; // The vehicle does not use this message
        // Resolve local MessageInformation
        Object newData[] = new Object[2];
        newData[0] = event.msg.sender.getString();
        newData[1] = event.msg.message;
        Message newMsg = new Message(msgInfo, newData, event.msg.msgLen);
        eesystem.simulator.addEvent(new MessageSendEvent(this, event.getEventTime().plus(properties.processing_time), newMsg));
    }

    @Override
    public Set<String> getUsedTags() {
        return tagSet;
    }

    @Override
    public void processTag(String tag, PortInformation portInfo) {
        // TODO handle multiple components with same port
        // Add the found ports as own ports
        if (portInfo.port_type != PortType.SOCKET)
            throw new IllegalArgumentException("Port " + portInfo.name + " has 'network' tag but is not of type 'SOCKET'.");
        if (!(portInfo.data_type instanceof SimplePacketType))
            throw new IllegalArgumentException("Port " + portInfo.name + " has 'network' tag but its DataType is not 'SimplePacketType'.");
        addPort(PortInformation.newSocketPort(portInfo.name, portInfo.data_type, portInfo.isOutput(), portInfo.isInput()));
        // Register msg types at the Network for type checking
        network.registerMessage(address, portInfo.name, portInfo.data_type);
    }

}
