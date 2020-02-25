/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.time.Instant;

/**
 * Interface for a discrete event in a discrete event simulation
 */
public abstract class DiscreteEvent {

    /**
     * Function that returns the time of the event
     *
     * @return Time of the event
     */
    public abstract Instant getEventTime();

    /**
     * Function that returns a numeric identifier for the event
     *
     * @return Numeric identifier for the event
     */
    public abstract int getEventId();
}
