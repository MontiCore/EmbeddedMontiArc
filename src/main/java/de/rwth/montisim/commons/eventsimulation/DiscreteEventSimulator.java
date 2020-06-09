/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation;

import java.time.Instant;
import java.util.PriorityQueue;

import de.rwth.montisim.commons.eventsimulation.exceptions.*;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.simulation.Updatable;

/**
 * Abstract class for a discrete event simulation based on scheduled events.
 */
public abstract class DiscreteEventSimulator<EventType, T extends DiscreteEvent<EventType>> implements Updatable {

    /** Comparator for events. Sorts in ascending order of event time. */
	private static final DiscreteEvent.DiscreteEventComparator listComparator = new DiscreteEvent.DiscreteEventComparator();

	/** point of time of simulation */
	private Instant simulationTime = Instant.EPOCH;

    /** list of all discrete events */
    private final PriorityQueue<T> eventList = new PriorityQueue<>(listComparator);

    /**
     * Function that needs to be implemented in subclasses of DiscreteEventSimulator
     * for processing events
     *
     * @param event Event to be processed
     */
    protected abstract void processEvent(T event);

    /**
     * function that adds an event to be scheduled
     * 
     * @param event event to add
     */
    public void addEvent(T event) {
        if (event.getEventTime().isBefore(simulationTime)){
            throw new InvalidEventException(event, simulationTime);
        }
		eventList.offer(event);
	}

    /**
     * Is called after the simulation objects updated their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime         Total simulation time in milliseconds
     * @param deltaTime         Delta simulation time in milliseconds
     */
    @ Override
    public void update(TimeUpdate newTime) {
        T cur;
		//loop until eventList is empty or next event is in future (not isAfter)
		while(!eventList.isEmpty() && !eventList.peek().getEventTime().isAfter(newTime.newTime)){
			cur = eventList.poll();
			processEvent(cur);
			this.simulationTime = cur.getEventTime();
        }
        this.simulationTime = newTime.newTime;
    }

    public Instant getSimulationTime() {
		return simulationTime;
    }
    
    public PriorityQueue<T> getEventList() {
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

}
