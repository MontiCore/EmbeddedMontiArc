/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation;

import java.time.Instant;
import java.util.PriorityQueue;

import de.rwth.montisim.commons.eventsimulation.exceptions.*;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;
import de.rwth.montisim.commons.utils.BuildObject;

/**
 * Abstract class for a discrete event simulation based on scheduled events.
 */
public class DiscreteEventSimulator implements Updatable, BuildObject {
    public static final String CONTEXT_KEY = "event_simulator";

    /** Comparator for events. Sorts in ascending order of event time. */
	private static final DiscreteEvent.DiscreteEventComparator listComparator = new DiscreteEvent.DiscreteEventComparator();

	/** point of time of simulation */
	protected Instant simulationTime;

    /** list of all discrete events */
    protected final PriorityQueue<DiscreteEvent> eventList = new PriorityQueue<>(listComparator);

    public DiscreteEventSimulator(Instant startTime) {
        this.simulationTime = startTime;
    }

    /**
     * function that adds an event to be scheduled
     * 
     * @param event event to add
     */
    public void addEvent(DiscreteEvent event) {
        if (event.getEventTime().isBefore(simulationTime)){
            throw new InvalidEventException(event, simulationTime);
        }
		eventList.offer(event);
	}

    @Override
    public void update(TimeUpdate newTime) {
        DiscreteEvent cur;
		//loop until eventList is empty or next event is in future (not isAfter)
		while(!eventList.isEmpty() && !eventList.peek().getEventTime().isAfter(newTime.newTime)){
            cur = eventList.poll();
            if (!cur.invalid)
                cur.target.process(cur);
			this.simulationTime = cur.getEventTime();
        }
        this.simulationTime = newTime.newTime;
    }

    public Instant getSimulationTime() {
		return simulationTime;
    }
    
    public PriorityQueue<DiscreteEvent> getEventList() {
		return eventList;
    }

    /**
     * Improved toString() method to get more information
     *
     * @return String of information about object
     */
    @Override
    public String toString() {
        return "DiscreteEventSimulator{" + "simTime=" + simulationTime + ", events=" + eventList+ '}';
    }

    @Override
    public String getKey() {
        return CONTEXT_KEY;
    }

}
