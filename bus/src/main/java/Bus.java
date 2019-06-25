
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
import java.util.PriorityQueue;
import java.util.Stack;
import java.util.UUID;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;

import commons.simulation.DiscreteEvent;

public abstract class Bus extends EEComponent {

	protected String ID;

	protected List<EEComponent> connectedComponents;

	protected Instant currentTime = Instant.EPOCH;

	protected String currentKeepAliveID = "";

	protected Bus(EESimulator simulator, List<EEComponent> connectedComponents) {
		super(simulator);
		if (connectedComponents.isEmpty()) {
			Log.info("Bus with no connected components initalized");
		}
		for (EEComponent comp : connectedComponents) {
			if (comp.simulator != simulator) {
				throw new IllegalArgumentException("Can not connect component from different simulator to bus");
			}
		}
		this.connectedComponents = connectedComponents;
		this.ID = "Bus" + UUID.randomUUID().toString();
	}

	public String getID() {
		return this.ID;
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
		KeepAliveEvent keepAlive = new KeepAliveEvent(this.getNextFinishTime());
		currentKeepAliveID = keepAlive.getEventId();
		this.simulator.addEvent(keepAlive);
	}

	protected void registerEventAtSimulator(BusMessage msg) {
		// TODO makes this sense?
		if(!msg.getNextHop().isPresent()) {
			throw new IllegalArgumentException("Next hop of msg can not be null");
		}
		else {
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
					msg.getfinalTarget().getID());
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

	abstract void simulateFor(Duration duration);

	abstract Instant getNextFinishTime();

	abstract void registerMessage(BusMessage msg);

}
