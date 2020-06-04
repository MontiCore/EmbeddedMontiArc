/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation.exceptions;

/**
 * Super class for all Event related exceptions. These are runtime exceptions, must not necessarily be caught.
 */
public abstract class EventException extends IllegalStateException {
    private static final long serialVersionUID = 1811089291721590662L;
    EventException(String desc){
        super(desc);
    }
}