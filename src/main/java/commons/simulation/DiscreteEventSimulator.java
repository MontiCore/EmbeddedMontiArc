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

import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

/**
 * Abstract class for a discrete event simulation based on scheduled events
 * This class implements the SimulationLoopNotifiable interface to be combined with vehicle simulator
 */
public abstract class DiscreteEventSimulator < DiscreteEventType extends DiscreteEvent > implements SimulationLoopNotifiable {

    /** Current total time in the discrete event simulation measured in nanoseconds */
    private long simulationTimeNs = 0;

    /** Last amount of time by which the simulation was advanced measured in nanoseconds */
    private long lastSimulationDeltaTimeNs = 0;

    /** List of discrete events */
    private final List<DiscreteEventType> eventList = Collections.synchronizedList(new LinkedList<>());

    /** List of simulation notifiable objects */
    private final List<DiscreteEventSimulationNotifiable> discreteEventSimulationNotifiableList = Collections.synchronizedList(new LinkedList<>());

    /**
     * Function that schedules a new event in the discrete event simulation
     *
     * @param event Event to be scheduled
     * @throws IllegalArgumentException when event is timed in the past
     */
    public void scheduleEvent(DiscreteEventType event) throws IllegalArgumentException {
        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.onScheduleEvent(event);
            }
        }

        // Throw exception if event is in the past
        if (event.getEventTime() < simulationTimeNs) {
            throw new IllegalArgumentException("scheduleEvent: Event is timed in the past! Event: " + event + ", simulationTimeNs: " + simulationTimeNs);
        }

        // Find correct position in list to add event
        int listIndex = 0;

        synchronized (eventList) {
            for (DiscreteEventType e : eventList) {
                // Stop increasing index if event in list is really later than new event
                if (e.getEventTime() > event.getEventTime()) {
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
    private void advanceSimulationTime(long deltaTime) {
        // Inform notifiable objects
        synchronized (discreteEventSimulationNotifiableList) {
            for (DiscreteEventSimulationNotifiable notifiable : discreteEventSimulationNotifiableList) {
                notifiable.onAdvanceSimulationTime(deltaTime);
            }
        }

        // Store last delta time value
        lastSimulationDeltaTimeNs = deltaTime;

        // Store initial time
        long initialSimulationTimeNs = simulationTimeNs;

        // Handle all events that are timed before new simulation time
        long targetTime = initialSimulationTimeNs + deltaTime;
        Optional<DiscreteEventType> event = getNextEventInTime(targetTime);
        while (event.isPresent()) {
            eventList.remove(0);

            // Advance time step by step and process event
            simulationTimeNs = event.get().getEventTime();

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

            event = getNextEventInTime(targetTime);
        }

        // Set simulation time to the end of time advancement
        simulationTimeNs = initialSimulationTimeNs + deltaTime;

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
    private Optional<DiscreteEventType> getNextEventInTime(long finalTime) {
        Optional<DiscreteEventType> result = Optional.empty();

        if (!eventList.isEmpty()) {
            DiscreteEventType event = eventList.get(0);

            // Check if time of first event matches
            if (event.getEventTime() <= finalTime) {
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
    protected void processEvent(DiscreteEventType event){

    }

    public void visitEvent(DiscreteEventType event){

    }

    /**
     * Get the current total discrete simulation time in nanoseconds
     *
     * @return Current total discrete simulation time in nanoseconds
     */
    public long getSimulationTimeNs() {
        return simulationTimeNs;
    }

    /**
     * Set the current total discrete simulation time in nanoseconds
     *
     * @param simulationTimeNs New total discrete simulation time in nanoseconds
     */
    protected void setSimulationTimeNs(long simulationTimeNs) {
        this.simulationTimeNs = simulationTimeNs;
    }

    /**
     * Get list of discrete events
     *
     * @return List of discrete events
     */
    public List<DiscreteEventType> getEventList() {
        return Collections.synchronizedList(new LinkedList<>(eventList));
    }

    /**
     * Set list of discrete events
     *
     * @param eventList New discrete event list
     */
    protected void setEventList(List<DiscreteEventType> eventList) {
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
     * Function that returns lastSimulationDeltaTimeNs
     *
     * @return Value for lastSimulationDeltaTimeNs
     */
    public long getLastSimulationDeltaTimeNs() {
        return lastSimulationDeltaTimeNs;
    }

    /**
     * Is called after the simulation objects updated their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    @Override
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {
        advanceSimulationTime(deltaTime * 1000000L);
    }

    /**
     * Is called just before the simulation objects update their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    @Override
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    /**
     * Is called for each object just before the simulation objects update their states.
     *
     * @param simulationObject Object for which the loop will be executed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    @Override
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    /**
     * Is called for each object after the simulation objects updated their states.
     *
     * @param simulationObject Object for which the loop iteration has been completed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    @Override
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

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
     * @param totalTime Total simulation time in milliseconds
     */
    @Override
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {}

    /**
     * Improved toString() method to get more information
     *
     * @return String of information about object
     */
    @Override
    public String toString() {
        return "DiscreteEventSimulator{" +
                "simulationTimeNs=" + simulationTimeNs +
                ", eventList=" + eventList +
                ", discreteEventSimulationNotifiableList=" + discreteEventSimulationNotifiableList +
                '}';
    }
}