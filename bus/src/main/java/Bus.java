
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
import java.util.List;
import java.util.PriorityQueue;
import java.util.UUID;


import commons.simulation.DiscreteEvent;



public abstract class Bus extends EEComponent{
	
	protected String ID;
	
	protected List<EEComponent> connectedComponents;
	
	protected Instant currentTime;
	
	protected String currentKeepAliveID;
	
	public Bus(EESimulator simulator, List<EEComponent> connectedComponents) {
		super(simulator);
		this.connectedComponents = connectedComponents;
		this.ID = "Bus" + UUID.randomUUID().toString();
	}
	
	public String getID() {
		return this.ID;
	}

	public void processEvent(DiscreteEvent evt) {
		if(evt instanceof BusMessage) {
			BusMessage msg = (BusMessage)evt;
			if(msg.getType() == MessageType.SEND) {
				this.simulateFor(Duration.between(currentTime, msg.getEventTime()));
				currentTime = msg.getEventTime();
				this.registerEvent(msg);
				this.registerMessage(msg);
				this.setKeepAlive();
			}
			else{
				throw new IllegalArgumentException("Invalid MessageType. Expected SEND but was " + msg.getType().toString());
			}
		}
		else if(evt instanceof KeepAliveEvent) {
			if(evt.getEventId() == this.currentKeepAliveID) {
				this.simulateFor(Duration.between(currentTime, evt.getEventTime()));
				currentTime = evt.getEventTime();
				this.setKeepAlive();
			}
		}
		else {
			throw new IllegalArgumentException("Invalid event type. Expected KeepAliveEvent or BusMessage but was " + evt.getClass());
		}
		
	}
	
	void setKeepAlive() {
		KeepAliveEvent keepAlive = new KeepAliveEvent(this.getNextFinishTime());
		currentKeepAliveID = keepAlive.getEventId();
		this.simulator.addEvent(keepAlive);
	}
	
	void registerEvent(BusMessage msg) {
		if(msg.getTarget() instanceof Bus) {
			msg.forwardToBus(this.ID);
			this.simulator.addEvent(msg);
		}
		else {
			msg.setType(MessageType.RECEIVE);
			this.simulator.addEvent(msg);
		}
	}
	
	abstract void simulateFor(Duration duration);

	abstract Instant getNextFinishTime();
	
	abstract void registerMessage(BusMessage msg); 

}
