/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation.exceptions;

import de.rwth.montisim.commons.eventsimulation.*;

import java.time.Instant;

/**
 * Exception thrown when a discrete event is registered or processed "in the
 * past".
 */
public class InvalidEventException extends EventException {
    private static final long serialVersionUID = 1811089291721590662L;

    public InvalidEventException(DiscreteEvent e, Instant simulationTime) {
        super("Event "+e+" lies in the past (sim-time: "+simulationTime+", event-time: "+e.getEventTime()+").");
    }
}