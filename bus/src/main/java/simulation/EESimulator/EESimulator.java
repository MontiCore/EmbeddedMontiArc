/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;


import simulation.bus.Bus;

import java.time.Duration;
import java.time.Instant;
import java.util.*;
import java.util.Comparator;

import org.jfree.util.Log;
import simulation.bus.BusMessage;

/**
 * Discrete event simulator that takes care of the communication in the car.
 */
public class EESimulator {

	/** Comparator for events. Sorts in ascending order of event time*/
	private static final EESimulatorComparator listComparator = new EESimulatorComparator();

	/** point of time of simulation */
	private Instant simulationTime;

	/**
	 * list of all discrete events
	 */
	private PriorityQueue<EEDiscreteEvent> eventList = new PriorityQueue<>(listComparator);


	public EESimulator(Instant simulationTime) {
		this.simulationTime = simulationTime;
	}

	/**
	 * function that adds event to eventList
	 * 
	 * @param event event to add
	 */
	public void addEvent(EEDiscreteEvent event) {
		if(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
			BusMessage msg = (BusMessage)event;
			Log.info(msg.getControllerID() + " registered: " + msg.getMessage() + " for "
					+ msg.getTarget().getId() + " at time " + msg.getEventTime());
		}
		eventList.offer(event);
	}

	/**
	 * simulate until eventList is empty or next event is in future
	 */
	public void simulateNextTick(Instant actualTime) {
		EEDiscreteEvent cur;
		//loop until eventList is empty or next event is in future (not isAfter)
		while(!eventList.isEmpty() && !eventList.peek().getEventTime().isAfter(actualTime)){
			cur = eventList.poll();
			
			Object msg = "";
			if(cur.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
				msg = ((BusMessage)cur).getMessage();
			}
			
			Log.info("----------------------------");
			Log.info("process event: " + msg + " " + cur.getEventType() + " at time "
					+ Duration.between(Instant.EPOCH, cur.getEventTime()).toNanos() + " for " 
					+ cur.getTarget().getId());

			cur.getTarget().processEvent(cur);
			this.simulationTime = cur.getEventTime();
			Log.info("----------------------------");
		}
	}

	public Instant getSimulationTime() {
		return simulationTime;
	}

	public PriorityQueue<EEDiscreteEvent> getEventList() {
		return eventList;
	}
}

/**
 *  Used for sorting in ascending order of event time
 */
class EESimulatorComparator implements Comparator<EEDiscreteEvent>
{
    public int compare(EEDiscreteEvent a, EEDiscreteEvent b)
    {
        return a.getEventTime().compareTo(b.getEventTime());
    }
}