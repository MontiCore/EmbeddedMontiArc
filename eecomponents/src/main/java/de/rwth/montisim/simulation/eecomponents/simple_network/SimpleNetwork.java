/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.util.HashMap;
import java.util.Vector;
import java.util.stream.Stream;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.eventsimulation.EventTarget;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.commons.StaticObject;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eecomponents.simple_network.events.*;

public class SimpleNetwork implements EventTarget, SimulatorModule {
    public static final String CONTEXT_KEY = "simple_network";

    // Used to address 'other cars' in the simulation: 
    // '2::1' is the 'first other car' (other than sender)
    // '2::2' is the 'second other car'
    public static final String N_TO_N_PREFIX = "2::";
    public static final String N_TO_N_BROADCAST_ADDR = "ff02::2";

    public final SimpleNetworkProperties properties;

    final DiscreteEventSimulator simulator;

    Vector<SimpleNetworkNodeInfo> nodes = new Vector<>();
    HashMap<String, SimpleNetworkNodeInfo> ipToNode = new HashMap<>();
    HashMap<String, Integer> ipToId = new HashMap<>();

    int addrCounter = 1;

    HashMap<String, Pair<DataType, IPV6Address>> msgTypes = new HashMap<>();

    public SimpleNetwork(SimpleNetworkProperties properties, DiscreteEventSimulator simulator) {
        this.properties = properties;
        this.simulator = simulator;
    }

    public IPV6Address registerNode(String nodeName, StaticObject physicalObject, SimpleCommunicationGateway component) {
        String addrStr = "1::" + Integer.toHexString(addrCounter);
        IPV6Address addr = new IPV6Address(addrStr);
        SimpleNetworkNodeInfo nodeInfo = new SimpleNetworkNodeInfo(nodeName, addr, physicalObject, component);
        int id = nodes.size();
        nodes.add(nodeInfo);
        ipToNode.put(addrStr, nodeInfo);
        ipToId.put(addrStr, id);
        addrCounter++;
        return addr;
    }

    public void registerMessage(IPV6Address addr, String msgName, DataType msgType) {
        Pair<DataType, IPV6Address> prev = msgTypes.get(msgName);
        if (prev == null) {
            msgTypes.put(msgName, new Pair<>(msgType, addr));
        } else {
            if (!prev.getKey().equals(msgType)) {
                SimpleNetworkNodeInfo prevNode = ipToNode.get(prev.getValue().getString());
                SimpleNetworkNodeInfo newNode = ipToNode.get(addr.getString());
                throw new IllegalArgumentException(
                        "Network Msg type error: Node " + newNode.nodeName + " registered msg '" + msgName + "' with type '" + msgType
                                + " which was already registered with type '" + prev.getKey() + "' by node " + prevNode.nodeName);
            }
        }
    }

    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == SimpleNetworkRecvEvent.type) {
            simulator.addEvent(new SimpleNetworkSendEvent(this, event.getEventTime().plus(properties.transmission_time), ((SimpleNetworkRecvEvent) event).msg));
        } else if (type == SimpleNetworkSendEvent.type) {
            dispatch((SimpleNetworkSendEvent) event);
        } else throw new UnexpectedEventException(this.toString(), event);
    }

    private void dispatch(SimpleNetworkSendEvent event) {
        SimpleNetworkNodeInfo sender = ipToNode.get(event.msg.sender.getString());
        SimpleNetworkRecvEvent recvEvent = new SimpleNetworkRecvEvent(null, event.getEventTime(), event.msg);
        if (event.msg.target.getString().equals(IPV6Address.BROADCAST_ADDR.getString())) {
            nodesInRange(sender).forEach(v -> {
                if (v != sender)
                    v.component.process(recvEvent);
            });
        } else if (event.msg.target.getString().equals(N_TO_N_BROADCAST_ADDR)) { // TODO this is hacky
            // N-to-N broadcast
            int senderId = ipToId.get(event.msg.sender.getString()) - 1; // When the sender is after the target in the 'nodes' table, the relative id is one smaller
            for (SimpleNetworkNodeInfo node : nodes) {
                if (node == sender) {
                    ++senderId; // Now target is after sender: id of sender is position in 'nodes' table.
                    continue;
                }
                recvEvent.msg.sender = new IPV6Address(N_TO_N_PREFIX + Integer.toString(senderId + 1)); // Convert sender ip to relative n-to-n address
                node.component.process(recvEvent);
            }
        } else if (event.msg.target.getString().startsWith(N_TO_N_PREFIX)) { // TODO this is hacky
            // N-to-N message
            int senderIndex = ipToId.get(event.msg.sender.getString());
            int targetId = Integer.parseInt(event.msg.target.getString().substring(N_TO_N_PREFIX.length())) - 1;
            if (targetId + 1 >= nodes.size()) {
                throw new IllegalArgumentException("Trying to send N-to-N message to invalid IP: " + event.msg.target.getString());
            }
            // Get 'other' target
            int targetIndex = targetId >= senderIndex ? targetId + 1 : targetId;
            SimpleNetworkNodeInfo target = nodes.elementAt(targetIndex);
            int senderId = senderIndex < targetId ? senderIndex : senderIndex - 1;
            recvEvent.msg.sender = new IPV6Address(N_TO_N_PREFIX + Integer.toString(senderId + 1)); // Convert sender ip to relative n-to-n address
            target.component.process(recvEvent);
            return;
        } else {
            SimpleNetworkNodeInfo target = ipToNode.get(event.msg.target.getString());
            if (target == null)
                throw new IllegalArgumentException("Sending to Unknown IP: " + event.msg.target.getString());
            if (inRange(target, sender)) {
                target.component.process(recvEvent);
            }
        }
    }

    private Stream<SimpleNetworkNodeInfo> nodesInRange(SimpleNetworkNodeInfo target) {
        return nodes.stream().filter(v -> inRange(target, v));
    }

    private boolean inRange(SimpleNetworkNodeInfo a, SimpleNetworkNodeInfo b) {
        return a.physicalObject.pos.distance(b.physicalObject.pos) < properties.car_transmission_range;
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }

}
