package simulation.bus;


import java.time.Duration;
import java.time.Instant;
import java.util.*;

import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;

import commons.simulation.DiscreteEvent;
import commons.controller.commons.BusEntry;

import commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;
import simulation.EESimulator.*;

public abstract class Bus extends EEComponent {

	protected String ID;

	protected List<EEComponent> connectedComponents;

	protected Instant currentTime = Instant.EPOCH;

	protected String currentKeepAliveID = "";

	protected HashMap<BusEntry, EEComponent> targetsOfComponents = new HashMap<>();

	protected Bus(EESimulator simulator, List<EEComponent> connectedComponents) {
		super(simulator);
		this.connectedComponents = new ArrayList<EEComponent>();
		//sendTo = new HashMap<BusEntry, List<EEComponent>>();
		this.ID = "Bus" + UUID.randomUUID().toString();
		componentType = EEComponentType.BUS;
	}

	public String getID() {
		return this.ID;
	}
	
	public void setCurrentTime(Instant currentTime) {
		this.currentTime = currentTime;
	}

	public void processEvent(EEDiscreteEvent evt) {
		if (evt instanceof BusMessage) {
			BusMessage msg = (BusMessage) evt;
			if (msg.getType() == MessageType.SEND) {
				this.simulateFor(Duration.between(currentTime, msg.getEventTime()));
				currentTime = msg.getEventTime();
				this.registerMessage(msg);
				this.setKeepAlive();
			} else {
				throw new IllegalArgumentException(
						"Invalid MessageType. Expected SEND but was " + msg.getType().toString());
			}
		} else if (evt instanceof KeepAliveEvent) {
			if (((KeepAliveEvent) evt).getEventId().equals(this.currentKeepAliveID)) {
				this.simulateFor(Duration.between(currentTime, evt.getEventTime()));
				currentTime = evt.getEventTime();
				this.setKeepAlive();
			}
		} else {
			throw new IllegalArgumentException(
					"Invalid event type. Expected KeepAliveEvent or BusMessage but was " + evt.getClass());
		}

	}

	protected void setKeepAlive() {
		KeepAliveEvent keepAlive = new KeepAliveEvent(this, this.getNextFinishTime(), this);
		currentKeepAliveID = keepAlive.getEventId();
		this.simulator.addEvent(keepAlive);
	}

	public void registerComponent(EEComponent component, List<BusEntry> messages){//messages are all needed/wanted messageIDs
		for(EEComponent connect : connectedComponents){
			if(connect.getComponentType() == EEComponentType.BRIDGE){
				((Bridge)connect).update(this, listenTo);
			}
		}
		connectedComponents.add(component);
		for(BusEntry message: messages){
			if(sendTo.containsKey(message)){
				List<EEComponent> targets = sendTo.get(message);
				targets.add(component);
				sendTo.put(message, targets);
			}
			else{
				List<EEComponent> list = new ArrayList<EEComponent>();
				list.add(component);
				sendTo.put(message, list);
				listenTo.add(message);
			}
		}
	}



	protected void updateSendTo(Bridge component, List<BusEntry> listenTo){
		for(EEComponent connect : connectedComponents){
			if(connect.getComponentType() == EEComponentType.BRIDGE && !connect.equals(component)){
				((Bridge)connect).update(this, listenTo);
			}
		}
		for(BusEntry message: listenTo){
			if(sendTo.containsKey(message)){
				List<EEComponent> targets = sendTo.get(message);
				targets.add(component);
				sendTo.put(message, targets);
			}
			else{
				List<EEComponent> list = new ArrayList<EEComponent>();
				list.add(component);
				sendTo.put(message, list);
			}
		}
	}

	abstract void simulateFor(Duration duration);

	abstract Instant getNextFinishTime();

	abstract void registerMessage(BusMessage msg);

}
