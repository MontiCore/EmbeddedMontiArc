/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation.exceptions;

import de.rwth.montisim.commons.eventsimulation.*;

/**
 * Exception thrown by Event processors if they don't expect events of the specific event type.
 */
public class UnexpectedEventException extends EventException {
    private static final long serialVersionUID = -4576798259802685210L;

    public UnexpectedEventException(String recipientName, DiscreteEvent e) {
        super(recipientName + " received event of type " + e.getClass().getSimpleName());
    }
}