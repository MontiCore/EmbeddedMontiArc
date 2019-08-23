/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

/**
 * Interface for a discrete event in a discrete event simulation
 */
public abstract class DiscreteEvent {

    /**
     * Function that returns the time of the event
     *
     * @return Time of the event
     */
    public abstract long getEventTime();

    /**
     * Function that returns a numeric identifier for the event
     *
     * @return Numeric identifier for the event
     */
    public abstract int getEventId();
}
