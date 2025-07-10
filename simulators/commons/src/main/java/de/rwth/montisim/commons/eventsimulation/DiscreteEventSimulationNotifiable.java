/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation;

import java.time.Duration;

/**
 * Interface for objects that want to get notified by a discrete event
 * simulation about specified actions
 */
public interface DiscreteEventSimulationNotifiable {
    /**
     * Function that is called when a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void onScheduleEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void afterScheduleEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called when the processing function for a discrete event is
     * called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void onProcessEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after the processing function for a discrete event is
     * called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void afterProcessEvent(DiscreteEvent discreteEvent);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time
     */
    public void onAdvanceSimulationTime(Duration deltaTime);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time
     */
    public void afterAdvanceSimulationTime(Duration deltaTime);

}
