/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

/**
 * Interface for objects that want to get notified by a discrete event simulation about specified actions
 */
public interface DiscreteEventSimulationNotifiable <T extends DiscreteEvent> {
    /**
     * Function that is called when a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void onScheduleEvent(T discreteEvent);

    /**
     * Function that is called after a discrete event is scheduled
     *
     * @param discreteEvent Discrete event that is scheduled
     */
    public void afterScheduleEvent(T discreteEvent);

    /**
     * Function that is called when the processing function for a discrete event is called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void onProcessEvent(T discreteEvent);

    /**
     * Function that is called after the processing function for a discrete event is called
     *
     * @param discreteEvent Discrete event that is processed
     */
    public void afterProcessEvent(T discreteEvent);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time in nanoseconds
     */
    public void onAdvanceSimulationTime(long deltaTime);

    /**
     * Function that is called after the simulation time is advanced
     *
     * @param deltaTime Delta simulation time in nanoseconds
     */
    public void afterAdvanceSimulationTime(long deltaTime);

}
