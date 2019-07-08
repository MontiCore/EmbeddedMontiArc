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
package simulation.bus;


import java.time.Duration;
import java.time.Instant;
import java.util.*;

import java.util.HashMap;
import commons.controller.commons.BusEntry;

import simulation.EESimulator.*;

public abstract class Bus extends EEComponent {

	protected UUID ID;

	protected List<EEComponent> connectedComponents;

	protected Instant currentTime = Instant.EPOCH;

	protected Bus(EESimulator simulator) {
		super(simulator);
		this.connectedComponents = new ArrayList<EEComponent>();
		this.ID = UUID.randomUUID();
		componentType = EEComponentType.BUS;
	}
	
	@Override
	public UUID getID() {
		return this.ID;
	}
	
	public void setCurrentTime(Instant currentTime) {
		this.currentTime = currentTime;
	}
	
	protected List<EEComponent> getConnectedComponents() {
		return this.connectedComponents;
	}

	public void processEvent(EEDiscreteEvent evt) {
		if (evt.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
			BusMessage msg = (BusMessage) evt;
			this.simulateFor(Duration.between(currentTime, msg.getEventTime()));
			currentTime = msg.getEventTime();
			if(!this.hasMessages()) {
				this.setKeepAlive();
			}
			this.registerMessage(msg);
			this.setKeepAlive();
		} else if (evt.getEventType() == EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT) {
			this.simulateFor(Duration.between(currentTime, evt.getEventTime()));
			currentTime = evt.getEventTime();
			this.setKeepAlive();
		} else {
			throw new IllegalArgumentException(
					"Invalid event type. Expected KeepAliveEvent or BusMessage but was " + evt.getClass());
		}

	}

	public void registerComponent(EEComponent component){
		if(connectedComponents.contains(component)) {
			throw new IllegalArgumentException("Component" + component.toString() + "is already registered at " + this.toString() + ".");
		}
		else {
			connectedComponents.add(component);
			for(BusEntry message: component.getListenTo()){
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
			for(EEComponent connect : connectedComponents){
				if(connect != component && connect.getComponentType() == EEComponentType.BRIDGE){
					((Bridge)connect).update(this, listenTo);
				}
			}
		}
		
	}



	public void updateSendTo(Bridge component, List<BusEntry> listenTo){
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
	
	protected void registerMessageAtSimulator(BusMessage msg) {
		for(EEComponent target : this.sendTo.get(msg.getMessageID())) {
			BusMessage newMsg = new BusMessage(msg);
			newMsg.forwardTo(target);
			this.simulator.addEvent(msg);
		}
	}
	
	protected void setKeepAlive() {
		KeepAliveEvent keepAlive = new KeepAliveEvent(this.getNextFinishTime(), this);
		this.simulator.addEvent(keepAlive);
	}

	abstract void simulateFor(Duration duration);

	abstract Instant getNextFinishTime();

	abstract void registerMessage(BusMessage msg);
	
	abstract boolean hasMessages();

}
