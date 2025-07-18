/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Vector;
import java.util.stream.Stream;

import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.EventTarget;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMissingComponentException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

public abstract class EEComponent implements EventTarget {
    public final transient EEComponentProperties properties;
    protected transient EESystem eesystem;
    public transient int id;

    public final transient List<EEComponent> connectedComponents = new ArrayList<>();
    public final transient HashMap<MessageInformation, List<EEComponent>> msgRoutingTable = new HashMap<>();

    // Ignored by buses & bridges
    protected final transient Vector<PortInformation> ports = new Vector<>();
    // MessageInformation for the messages SENT by this component
    protected final transient HashMap<String, MessageInformation> msgInfos = new HashMap<>();
    protected final transient HashMap<Integer, MessageInformation> msgInfoById = new HashMap<>();

    public EEComponent(EEComponentProperties properties, EESystem eesystem) {
        this.properties = properties;
        this.eesystem = eesystem;
        this.id = eesystem.registerComponent(this, properties.priority);
    }

    /**
     * Looks up the Component in the ComponentManager -> the Component must be
     * created first.
     */
    public void connectToComponent(String name) throws EEMissingComponentException {
        connectToComponent(this.eesystem.getComponent(name).orElseThrow(() -> new EEMissingComponentException(name)));
    }

    /**
     * Connects two components together for messaging.
     */
    public void connectToComponent(EEComponent component) {
        if (component == null)
            throw new IllegalArgumentException("Trying to connect '" + this + "' to missing component.");
        this.connectOneWay(component);
        component.connectOneWay(this);
    }

    private void connectOneWay(EEComponent component) {
        if (!connectedComponents.contains(component))
            connectedComponents.add(component);
    }


    // Default Event processing behavior: handle message events
    @Override
    public void process(DiscreteEvent event) {
        int type = event.getType();
        if (type == MessageSendEvent.type) {
            dispatchMessage((MessageSendEvent) event);
        } else if (type == MessageReceiveEvent.type) {
            receive((MessageReceiveEvent) event);
        } else
            throw new UnexpectedEventException(this.toString(), event);
    }

    /**
     * Dispatches the given Message to all its targets.
     */
    protected void dispatchMessage(MessageSendEvent msgSendEvent) {
        // NOTE: targets must exist but can be an empty list. (=> The Routing has to
        // have been computed for the EE system.)
        List<EEComponent> targets = msgRoutingTable.get(msgSendEvent.getMessage().msgInfo);
        if (targets == null) return;
        MessageReceiveEvent recvEvent = new MessageReceiveEvent(null, msgSendEvent.getEventTime(), msgSendEvent.getMessage());
        for (EEComponent e : targets) {
            e.process(recvEvent);
        }
    }

    protected abstract void receive(MessageReceiveEvent msgRecvEvent);


    /*
        Helper methods for application components (not buses or bridges)
    */

    public int addPort(PortInformation portInfo) {
        int msgId = eesystem.getMessageId(portInfo.name, portInfo.data_type);
        ports.add(portInfo);
        if (portInfo.isOutput()) {
            MessageInformation msgInf = new MessageInformation(msgId, portInfo.name, portInfo.data_type, this);
            msgInfos.put(portInfo.name, msgInf);
            msgInfoById.put(msgId, msgInf);
        }
        return msgId;
    }


    public void sendMessage(Instant time, int msgId, Object message, int msgLen) {
        this.eesystem.simulator.addEvent(new MessageSendEvent(this, time, new Message(msgInfoById.get(msgId), message, msgLen)));
    }

    public void sendMessage(Instant time, int msgId, Object message) {
        this.eesystem.simulator.addEvent(new MessageSendEvent(this, time, new Message(msgInfoById.get(msgId), message)));
    }

    public void sendMessage(Instant time, Message msg) {
        this.eesystem.simulator.addEvent(new MessageSendEvent(this, time, msg));
    }

    public Stream<PortInformation> streamInputPorts() {
        return ports.stream().filter(x -> x.isInput());
    }

    public Stream<PortInformation> streamOutputPorts() {
        return ports.stream().filter(x -> x.isOutput());
    }

    public MessageInformation getMsgInfo(String name) {
        return msgInfos.get(name);
    }

    @Override
    public String toString() {
        return "EEComponent \"" + properties.name + '"';
    }
}
