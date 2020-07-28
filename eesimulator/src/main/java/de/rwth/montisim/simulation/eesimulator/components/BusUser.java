/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;

public abstract class BusUser extends EEEventProcessor{
    public final transient BusUserProperties properties;

    public BusUser(BusUserProperties properties) {
        super(properties);
        this.properties = properties;
	}

	/**
     * Buses this component is connected to.
     */
    protected final transient List<Bus> connectedBuses = new ArrayList<>();
    
    protected final transient HashMap<Integer, List<Bus>> msgTargets = new HashMap<>();
    
    public List<Bus> getConnectedBuses(){
        return connectedBuses;
    }
    /**
     * This method connects the component to the BUS.
     * A Component can be connected to multiple buses.
     */
    public void connectToBus(Bus bus) {
        if (bus == null) return; // TODO EXCEPTION??
		if (connectedBuses.contains(bus))
			throw new IllegalArgumentException("Bus " + bus + " is already registered at " + this + ".");
        connectedBuses.add(bus);
        bus.addComponent(this);
    }
    /** Looks up the Bus in the ComponentManager -> the bus must be created first. */
    public void connectToBus(String name) {
        connectToBus(this.simulator.getComponentManager().getBus(name));
    }
    /** Looks up the Bus in the ComponentManager -> the bus must be created first. */
    public void connectToBus(int componentId) {
        connectToBus(this.simulator.getComponentManager().getBus(componentId));
    }

    public void addMessageTargets(int msgId, List<Bus> targets){
		if (msgTargets.containsKey(msgId)) throw new IllegalArgumentException("Targets already registered for msgId: " + msgId);
		msgTargets.put(msgId, targets);
    }
    
    /** Dispatches the given Message to all its targets. */
	protected void dispatchMessage(MessageSendEvent msgSendEvent) {
        // NOTE: targets must exist but can be an empty list. (=> The Routing has to have been computed for the EE system.)
		List<Bus> targets = msgTargets.get(msgSendEvent.getMessage().msgInfo.messageId);
		if (targets == null) throw new IllegalArgumentException("Tried to dispatch a message with no associated targets. (event: "+msgSendEvent+").");
		for(Bus e : targets){
			e.process(msgSendEvent);
        }
    }
}