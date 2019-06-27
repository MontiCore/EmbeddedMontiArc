
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

import java.time.Instant;
import java.util.*;
import java.util.Comparator;

import org.jfree.util.Log;

import commons.simulation.DiscreteEvent;

public class EESimulator {

	/** point of time of simulation */
	private Instant simulationTime;

	/** point of time of last simulation step */
	private Instant deltaSimulationTime;

	/*
	 * private long time = System.nanoTime(); speichert beim erstellen eine Zeit in
	 * ns. beim abfragen der Zeitdifferenz kann man dann (System.nanoTime() - time)
	 * ist ggf etwas eleganter
	 */

	/**
	 * list of discrete events (in the future)
	 */
	private static final EESimulatorComparator listComparator = new EESimulatorComparator();
	private PriorityQueue<DiscreteEvent> eventList = new PriorityQueue<>(listComparator);

	/**
	 * lists of objects that can get addressed by messages
	 */
	private List<EEComponent> busList = Collections.synchronizedList(new LinkedList<>());

	private List<EEComponent> sensorList = Collections.synchronizedList(new LinkedList<>());

	private List<EEComponent> actuatorList = Collections.synchronizedList(new LinkedList<>());

	public EESimulator(Instant simulationTime) {
		this.simulationTime = simulationTime;
		this.deltaSimulationTime = simulationTime;
	}

	/**
	 * function that adds event to eventList
	 * 
	 * @param event event to add
	 */

	public void addEvent(DiscreteEvent event) {
		eventList.offer(event);
	}

	/**
	 * simulate until eventList is empty or next event is in future
	 */
	public void simulateNextTick(Instant actualTime) {
		this.simulationTime = actualTime;
		
		//loop until eventList is empty or next event is in future
		while(!eventList.isEmpty() && eventList.peek().getEventTime().isAfter(simulationTime)){
			DiscreteEvent cur = eventList.poll(); 

			// check if event is type BusMessage or KeepAliveEvent
			if (cur instanceof BusMessage) {
				BusMessage msg = (BusMessage) cur;
				if (msg.getType() == MessageType.SEND) {
					if (busList.contains(msg.getNextHop().get())) {
						msg.getNextHop().get().processEvent(msg);
					}
					else {
						Log.warn("EESimulator processed event for unregistered bus");
					}
				} else if (msg.getType() == MessageType.RECEIVE) {
					msg.getTarget().processEvent(msg);
				}

			} else if (cur instanceof KeepAliveEvent) {
				KeepAliveEvent keepAlive = (KeepAliveEvent)cur;
				if (busList.contains(keepAlive.getBus())) {
					keepAlive.getBus().processEvent(keepAlive);
				}
				else {
					Log.warn("EESimulator processed event for unregistered bus");
				}
			}
		}

	}

	/**
	 * function that adds component to listenerList
	 * 
	 * @param listener component to add
	 */
	public void registerComponent(EEComponent listener) { // sensor and actuator have to be instance of EEComponent
		if (listener instanceof Bus) {
			busList.add(listener);
		} /*
			 * else if (listener instanceof VehicleActuator) { actuatorList.add(listener); }
			 * else if (listener instanceof AbstractSensor || listener instanceof
			 * AbstractDistanceSensor){ sensorList.add(listener); }
			 */
		else {
			return;
		}
	}

	public Instant getDeltaSimulationTime() {
		return deltaSimulationTime;
	}

	public Instant getSimulationTime() {
		return simulationTime;
	}

}

class EESimulatorComparator implements Comparator<DiscreteEvent> {
	// Used for sorting in ascending order of event time
	public int compare(DiscreteEvent a, DiscreteEvent b) {
		return a.getEventTime().compareTo(b.getEventTime());
	}
}
