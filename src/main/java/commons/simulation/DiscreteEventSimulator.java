/**
 *
 *  ******************************************************************************
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
package commons.simulation;

import java.time.Duration;
import java.time.Instant;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

/**
 * Abstract class for a discrete event simulation based on scheduled events
 * This class implements the SimulationLoopNotifiable interface to be combined with vehicle simulator
 */
public abstract class DiscreteEventSimulator implements SimulationLoopNotifiable {

    /** Current total time in the discrete event simulation */
    private Instant simulationTime = Instant.EPOCH;

    /** Last amount of time by which the simulation was advanced */
    private Duration lastSimulationDeltaTime = Duration.ZERO;

    /** List of discrete events */
    private final List<DiscreteEvent> eventList = Collections.synchronizedList(new LinkedList<>());

    /** List of simulation notifiable objects */
    private final List<DiscreteEventSimulationNotifiable> discreteEventSimulationNotifiableList = Collections.synchronizedList(new LinkedList<>());

    /**
     * Function that schedules a new event in the discrete event simulation
     *
     * @param event Event to be scheduled
     * @throws IllegalArgumentException when event is timed in the past
     */
    public void scheduleEvent(DiscreteEvent event) throws IllegalArgumentException {
        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.onScheduleEvent(event);
            }
        }

        // Throw exception if event is in the past
        if (event.getEventTime().isBefore(simulationTime)) {
            throw new IllegalArgumentException("scheduleEvent: Event is timed in the past! Event: " + event + ", simulationTime: " + simulationTime);
        }

        // Find correct position in list to add event
        int listIndex = 0;

        synchronized (eventList) {
            for (DiscreteEvent e : eventList) {
                // Stop increasing index if event in list is really later than new event
                if (e.getEventTime().isAfter(event.getEventTime())) {
                    break;
                }

                // Increase index position
                listIndex++;
            }
        }

        eventList.add(listIndex, event);

        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.afterScheduleEvent(event);
            }
        }
    }

    /**
     * Function that is called when the discrete simulation time progresses
     * Execute all discrete events in order that are in event list before the new total time
     *
     * @param deltaTime Delta simulation time in nanoseconds
     */
    private void advanceSimulationTime(Duration deltaTime) {
        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.onAdvanceSimulationTime(deltaTime);
            }
        }

        // Store last delta time value
        lastSimulationDeltaTime = deltaTime;

        // Store initial time
        Instant initialSimulationTime = Instant.from(simulationTime);
        
        Instant endSimulationTime = initialSimulationTime.plusNanos(deltaTime.toNanos());

        // Handle all events that are timed before new simulation time
        while (getNextEventInTime(endSimulationTime).isPresent()) {
            Optional<DiscreteEvent> event = getNextEventInTime(endSimulationTime);
            eventList.remove(0);

            // Advance time step by step and process event
            if (event.isPresent()) {
                simulationTime = event.get().getEventTime();

                // Inform notifiable objects
                synchronized (discreteEventSimulationNotifiableList) {
                    for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                        notifiable.onProcessEvent(event.get());
                    }
                }

                processEvent(event.get());

                // Inform notifiable objects
                synchronized (discreteEventSimulationNotifiableList) {
                    for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                        notifiable.afterProcessEvent(event.get());
                    }
                }
            }
        }

        // Set simulation time to the end of time advancement
        simulationTime = endSimulationTime;

        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.afterAdvanceSimulationTime(deltaTime);
            }
        }
    }

    /**
     * Function that computes the next event that can be executed in simulation time
     * Does not return events that are in the future and returns Optional.empty then
     *
     * @param finalTime Final time before which events should be returned
     * @return Next event ready for event handling
     */
    private Optional<DiscreteEvent> getNextEventInTime(Instant finalTime) {
        Optional<DiscreteEvent> result = Optional.empty();

        if (!eventList.isEmpty()) {
            DiscreteEvent event = eventList.get(0);

            // Check if time of first event matches
            if (!event.getEventTime().isAfter(finalTime)) {
                result = Optional.of(event);
            }
        }

        // Return computed result
        return result;
    }

    /**
     * Function that needs to be implemented in subclasses of DiscreteEventSimulator for processing events
     *
     * @param event Event to be processed
     */
    protected abstract void processEvent(DiscreteEvent event);

    /**
     * Get the current total discrete simulation time
     *
     * @return Current total discrete simulation time
     */
    public Instant getSimulationTime() {
        return simulationTime;
    }

    /**
     * Set the current total discrete simulation time
     *
     * @param simulationTime New total discrete simulation time
     */
    protected void setSimulationTime(Instant simulationTime) {
        this.simulationTime = simulationTime;
    }

    /**
     * Get list of discrete events
     *
     * @return List of discrete events
     */
    public List<DiscreteEvent> getEventList() {
        return Collections.synchronizedList(new LinkedList<>(eventList));
    }

    /**
     * Set list of discrete events
     *
     * @param eventList New discrete event list
     */
    protected void setEventList(List<DiscreteEvent> eventList) {
        this.eventList.clear();
        this.eventList.addAll(eventList);
    }

    /**
     * Function that adds a new discrete event simulation notifiable to the simulation
     *
     * @param simulationNotifiable Discrete event simulation notifiable to be added
     */
    public void registerDiscreteEventSimulationNotifiable(DiscreteEventSimulationNotifiable simulationNotifiable) {
        discreteEventSimulationNotifiableList.add(simulationNotifiable);
    }

    /**
     * Function that removes a discrete event simulation notifiable from the simulation
     *
     * @param simulationNotifiable Discrete event simulation notifiable to be removed
     */
    public void unregisterDiscreteEventSimulationNotifiable(DiscreteEventSimulationNotifiable simulationNotifiable) {
        discreteEventSimulationNotifiableList.remove(simulationNotifiable);
    }

    /**
     * Function that returns lastSimulationDeltaTime
     *
     * @return Value for lastSimulationDeltaTime
     */
    public Duration getLastSimulationDeltaTime() {
        return lastSimulationDeltaTime;
    }

    /**
     * Is called after the simulation objects updated their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {
        advanceSimulationTime(deltaTime);
    }

    /**
     * Is called just before the simulation objects update their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time
     * @param deltaTime Delta simulation time
     */
    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime, Duration deltaTime) {}

    /**
     * Is called for each object just before the simulation objects update their states.
     *
     * @param simulationObject Object for which the loop will be executed
     * @param totalTime Total simulation time
     * @param deltaTime Delta simulation time
     */
    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration deltaTime) {}

    /**
     * Is called for each object after the simulation objects updated their states.
     *
     * @param simulationObject Object for which the loop iteration has been completed
     * @param totalTime Total simulation time
     * @param deltaTime Delta simulation time
     */
    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime, Duration deltaTime) {}

    /**
     * Is called just before the simulation starts
     *
     * @param simulationObjects List of all simulation objects
     */
    @Override
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}

    /**
     * Is called just after the simulation ends
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time
     */
    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {}

    /**
     * Improved toString() method to get more information
     *
     * @return String of information about object
     */
    @Override
    public String toString() {
        return "DiscreteEventSimulator{" +
                "simulationTime=" + simulationTime +
                ", eventList=" + eventList +
                ", discreteEventSimulationNotifiableList=" + discreteEventSimulationNotifiableList +
                '}';
    }
}
