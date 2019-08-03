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

import java.time.Instant;
import java.util.*;

import commons.controller.commons.BusEntry;

import simulation.EESimulator.*;

public abstract class Bus extends MutableEEComponent {

	private List<EEComponent> connectedComponents = new ArrayList<EEComponent>();

	private Optional<KeepAliveEvent> keepAlive = Optional.empty();

	protected Bus(EESimulator simulator) {
		super(simulator, EEComponentType.BUS);
	}

	public void processEvent(EEDiscreteEvent evt) {
		if (evt.getEventTime().isBefore(this.getCurrentTime())) {
			System.out.println("Can not process event: Time of event already simulated! Last time was " + this.getCurrentTime());
			throw new IllegalArgumentException("Can not process event: Time of event already simulated! Last time was " + this.getCurrentTime());
		}
		if (evt.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
			BusMessage msg = (BusMessage) evt;
			// only work on messages if it was not already seen before
			if (!msg.hasTraveresed(this)) {
				this.simulateUntil(msg.getEventTime());
				this.registerMessage(msg);
				this.setKeepAlive();
			}
		} else if (evt.getEventType() == EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT) {
			if (this.keepAlive.get().getId() == evt.getId()) {
				this.simulateUntil(evt.getEventTime());
				if (this.hasMessages()) {
					this.setKeepAlive();
				} else {
					this.keepAlive = Optional.empty();
				}
			}
		} else {
			throw new IllegalArgumentException(
					"Invalid event type. Expected KeepAliveEvent or BusMessage but was " + evt.getEventType());
		}

	}

	public void registerComponent(EEComponent component) {
		if (connectedComponents.contains(component)) {
			throw new IllegalArgumentException(
					"Component" + component.toString() + "is already registered at " + this.toString() + ".");
		} else {
			connectedComponents.add(component);
			addSubscribedMessages(component, component.getSubscribedMessages());
		}

	}

	public void addSubscribedMessages(EEComponent component, List<BusEntry> messages) {
		List<BusEntry> newMessages = new ArrayList<BusEntry>();
		for (BusEntry message : messages) {
			List<EEComponent> targets = targetsByMessageId.getOrDefault(message, new ArrayList<EEComponent>());
			if(!targets.contains(component)) {
				targets.add(component);
				targetsByMessageId.put(message, targets);
				newMessages.add(message);
			}
			if (!subscribedMessages.contains(message)) {
				subscribedMessages.add(message);
			}
		}
		if(!newMessages.isEmpty()) {
			for (EEComponent connectedComponent : connectedComponents) {
				if (connectedComponent != component && connectedComponent.getComponentType() == EEComponentType.BRIDGE) {
					((Bridge) connectedComponent).update(this, newMessages);
				}
			}
		}

	}

	public abstract Instant getCurrentTime();

	public List<EEComponent> getConnectedComponents() {
		return this.connectedComponents;
	}

	protected void registerMessageAtSimulator(BusMessage msg) {
		for (EEComponent target : this.targetsByMessageId.get(msg.getMessageID())) {
			this.getSimulator().addEvent(msg.forwardTo(target));
		}
	}

	protected void setKeepAlive() {
		Instant nextFinishTime = this.getNextFinishTime();
		// at least one message present AND (old keepAlive empty OR old keepAlive
		// already processed OR new keepAlive
		// before old one)
		if (this.hasMessages()
				&& (!this.keepAlive.isPresent() || !keepAlive.get().getEventTime().isAfter(this.getCurrentTime())
						|| nextFinishTime.isBefore(keepAlive.get().getEventTime()))) {
			keepAlive = Optional.of(new KeepAliveEvent(nextFinishTime, this));
			this.getSimulator().addEvent(keepAlive.get());
			System.out.println("Bus: " + this.getId() + " has set new keepAliveEvent with time: " + nextFinishTime);
		}
	}

	protected abstract void simulateUntil(Instant endTime);

	protected abstract Instant getNextFinishTime();

	protected abstract void registerMessage(BusMessage msg);

	protected abstract boolean hasMessages();

}
