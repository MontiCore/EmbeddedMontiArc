package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.util.HashMap;
import java.util.Vector;
import java.util.stream.Stream;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.eventsimulation.EventTarget;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.commons.simulation.StaticObject;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eecomponents.simple_network.events.*;

public class SimpleNetwork implements EventTarget, SimulatorModule {
    public static final String CONTEXT_KEY = "simple_network";

    public final SimpleNetworkProperties properties;

    final DiscreteEventSimulator simulator;

    Vector<SimpleNetworkNodeInfo> nodes = new Vector<>();
    HashMap<String, SimpleNetworkNodeInfo> ipToNode = new HashMap<>();

    int addrCounter = 1;

    HashMap<String, Pair<DataType, IPV6Address>> msgTypes = new HashMap<>();

    public SimpleNetwork(SimpleNetworkProperties properties, DiscreteEventSimulator simulator) {
        this.properties = properties;
        this.simulator = simulator;
    }

    public IPV6Address registerNode(String nodeName, StaticObject physicalObject, SimpleCommunicationGateway component) {
        String addrStr = "1::"+Integer.toHexString(addrCounter);
        IPV6Address addr = new IPV6Address(addrStr);
        SimpleNetworkNodeInfo nodeInfo = new SimpleNetworkNodeInfo(nodeName, addr, physicalObject, component);
        nodes.add(nodeInfo);
        ipToNode.put(addr.addr, nodeInfo);
        addrCounter++;
        return addr;
    }

    public void registerMessage(IPV6Address addr, String msgName, DataType msgType) {
        Pair<DataType, IPV6Address> prev = msgTypes.get(msgName);
        if (prev == null) {
            msgTypes.put(msgName, new Pair<>(msgType, addr));
        } else {
            if (!prev.getKey().equals(msgType)) {
                SimpleNetworkNodeInfo prevNode = ipToNode.get(prev.getValue().addr);
                SimpleNetworkNodeInfo newNode = ipToNode.get(addr.addr);
                throw new IllegalArgumentException(
                    "Network Msg type error: Node "+newNode.nodeName + " registered msg '"+msgName+"' with type '"+msgType
                    +" which was already registered with type '"+prev.getKey()+"' by node "+prevNode.nodeName);
            }
        }
    }

    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == SimpleNetworkRecvEvent.type){
            simulator.addEvent(new SimpleNetworkSendEvent(this, event.getEventTime().plus(properties.transmission_time), ((SimpleNetworkRecvEvent)event).msg));
        } else if (type == SimpleNetworkSendEvent.type){
            dispatch((SimpleNetworkSendEvent) event);
        } else throw new UnexpectedEventException(this.toString(), event);
    }

    private void dispatch(SimpleNetworkSendEvent event) {
        SimpleNetworkNodeInfo sender = ipToNode.get(event.msg.sender.addr);
        SimpleNetworkRecvEvent recvEvent = new SimpleNetworkRecvEvent(null, event.getEventTime(), event.msg);
        if (event.msg.target.addr.equals(IPV6Address.BROADCAST_ADDR.addr)) {
            nodesInRange(sender).forEach(v -> {
                if (v != sender)
                    v.component.process(recvEvent);
            });
        } else {
            SimpleNetworkNodeInfo target = ipToNode.get(event.msg.target.addr);
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
