
/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.Stack;
import java.util.UUID;

import java.util.HashMap;
import java.util.Map;
import java.util.Iterator;
import java.util.Set;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;

import commons.simulation.DiscreteEvent;
import commons.controller.commons.BusEntry;

public abstract class Bus extends EEComponent {

	protected String ID;

	protected List<EEComponent> connectedComponents;

	protected Instant currentTime = Instant.EPOCH;

	protected String currentKeepAliveID = "";

	//protected HashMap<BusEntry, List<EEComponent>> sendTo;

	protected Bus(EESimulator simulator) {
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

	public void processEvent(DiscreteEvent evt) {
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
			if (evt.getEventId() == this.currentKeepAliveID) {
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
		KeepAliveEvent keepAlive = new KeepAliveEvent(this, this.getNextFinishTime());
		currentKeepAliveID = keepAlive.getEventId();
		this.simulator.addEvent(keepAlive);
	}

	protected void registerEventAtSimulator(BusMessage msg) {
		// TODO makes this sense?
		if (!msg.getNextHop().isPresent()) {
			throw new IllegalArgumentException("Next hop of msg can not be null");
		} else {
			if (msg.getNextHop().get() instanceof Bus) {
				msg.forwardToBus(this.ID);
				this.simulator.addEvent(msg);
			} else {
				msg.setType(MessageType.RECEIVE);
				this.simulator.addEvent(msg);
			}
		}
	}

	protected boolean setPath(BusMessage msg) {
		boolean res = false;
		Pair<Bus, List<EEComponent>> pair = new MutablePair<Bus, List<EEComponent>>(this, new ArrayList<EEComponent>());
		Stack<Pair<Bus, List<EEComponent>>> workStack = new Stack<Pair<Bus, List<EEComponent>>>();
		workStack.push(pair);
		while (!res && !workStack.isEmpty()) {
			Pair<Bus, List<EEComponent>> cur = workStack.pop();
			Optional<EEComponent> comp = BusUtils.findComponentWithID(cur.getLeft().connectedComponents,
					msg.getTarget().getID());
			if (comp.isPresent()) {
				cur.getRight().add(comp.get());
				msg.setPath(cur.getRight());
				res = true;
			} else {
				Iterable<Bus> connectedBuses = BusUtils.findConnectedBuses(cur.getLeft().connectedComponents);
				for (Bus connectedBus : connectedBuses) {
					ArrayList<EEComponent> path = new ArrayList<EEComponent>(cur.getRight());
					path.add(connectedBus);
					pair = new MutablePair<Bus, List<EEComponent>>(connectedBus, path);
					workStack.push(pair);
				}
			}
		}
		return res;
	}


	public void registerComponent(EEComponent component){
		List<BusEntry> messages = component.getListenTo();
		registerComponent(component, messages);
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
