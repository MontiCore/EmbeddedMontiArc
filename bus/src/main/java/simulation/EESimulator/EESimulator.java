package simulation.EESimulator;
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


import simulation.bus.Bus;

import java.time.Duration;
import java.time.Instant;
import java.util.*;
import java.util.Comparator;

import org.jfree.util.Log;
import simulation.bus.BusMessage;

public class EESimulator {

	/** point of time of simulation */
	private Instant simulationTime;

	/** point of time the simulator should simulate to, given by the hardware simulator  */
	private Instant deltaSimulationTime;


	/**
	 * list of all discrete events
	 */
	private static final EESimulatorComparator listComparator = new EESimulatorComparator();
	private PriorityQueue<EEDiscreteEvent> eventList = new PriorityQueue<>(listComparator);

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

	public void addEvent(EEDiscreteEvent event) {
		if(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
			BusMessage msg = (BusMessage)event;
			System.out.println(msg.getControllerID() + " registered: " + msg.getMessage() + " for " 
					+ msg.getTarget().getId() + " at time " + msg.getEventTime());
		}
		eventList.offer(event);
	}

	/**
	 * simulate until eventList is empty or next event is in future
	 */
	public void simulateNextTick(Instant actualTime) {
		this.deltaSimulationTime = actualTime;

		EEDiscreteEvent cur;
		//loop until eventList is empty or next event is in future
		while(!eventList.isEmpty() && !eventList.peek().getEventTime().isAfter(deltaSimulationTime)){
			cur = eventList.poll();
			
			Object msg = "";
			if(cur.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
				msg = ((BusMessage)cur).getMessage();
			}
			
			System.out.println("----------------------------");
			System.out.println("process event: " + msg + " " + cur.getEventType() + " at time " 
					+ Duration.between(Instant.EPOCH, cur.getEventTime()).toNanos() + " for " 
					+ cur.getTarget().getId());

			cur.getTarget().processEvent(cur);
			this.simulationTime = cur.getEventTime();
			System.out.println("----------------------------");
		}
	}

	/**
	 * function that adds component to listenerList
	 * 
	 * @param listener component to add
	 */
	public void registerComponent(EEComponent listener) { // sensor and actuator have to be instance of EESimulator
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

	public PriorityQueue<EEDiscreteEvent> getEventList() {
		return eventList;
	}
}

class EESimulatorComparator implements Comparator<EEDiscreteEvent>
{
    // Used for sorting in ascending order of event time
    public int compare(EEDiscreteEvent a, EEDiscreteEvent b)
    {
        return a.getEventTime().compareTo(b.getEventTime());
    }
}