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

import commons.simulation.DiscreteEvent;

public class EESimulator {

	/** point of time of simulation */
	private Instant simulationTimeNs;

	/** point of time of last simulation step */
	private Instant deltaSimulationTimeNs;

	/*
	 * private long time = System.nanoTime(); speichert beim erstellen eine Zeit in
	 * ns. beim abfragen der Zeitdifferenz kann man dann (System.nanoTime() - time)
	 * ist ggf etwas eleganter
	 */

	/**
	 * list of discrete events (in the future)
	 */
	private static final EESimulatorComparator listComparator = new EESimulatorComparator();
	private PriorityQueue<DiscreteEvent> eventList = new PriorityQueue<>(Integer.MAX_VALUE, listComparator);

	/**
	 * lists of objects that can get addressed by messages
	 */
	private List<EEComponent> busList = Collections.synchronizedList(new LinkedList<>());

	private List<EEComponent> sensorList = Collections.synchronizedList(new LinkedList<>());

	private List<EEComponent> actuatorList = Collections.synchronizedList(new LinkedList<>());

	public EESimulator(Instant simulationTimeNs) {
		this.simulationTimeNs = simulationTimeNs;
		this.deltaSimulationTimeNs = simulationTimeNs;
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
		this.simulationTimeNs = actualTime;
		for (DiscreteEvent event : eventList) {

			// check if event is in future
			if (event.getEventTime().isAfter(simulationTimeNs)) {
				this.deltaSimulationTimeNs = simulationTimeNs;
				return;
			}

			// check if event is type BusMessage or KeepAliveEvent
			if (event instanceof BusMessage) {

				if (((BusMessage) event).getType() == MessageType.SEND) {
					// TODO: change bus which is chosen
					busList.get(0).processEvent(event);
					eventList.remove(event);
				} else if (((BusMessage) event).getType() == MessageType.RECEIVE
						&& ((BusMessage) event).getFinishTime().isBefore(simulationTimeNs)) {
					((BusMessage) event).getTarget().processEvent(event);
					eventList.remove(event);
				}

			} else if (event instanceof KeepAliveEvent) {
				// TODO: implement processing KeepAliveEvents
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

	public Instant getDeltaSimulationTimeNs() {
		return deltaSimulationTimeNs;
	}

	public Instant getSimulationTimeNs() {
		return simulationTimeNs;
	}

}

class EESimulatorComparator implements Comparator<DiscreteEvent> {
	// Used for sorting in ascending order of event time
	public int compare(DiscreteEvent a, DiscreteEvent b) {
		return a.getEventTime().compareTo(b.getEventTime());
	}
}
