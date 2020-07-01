/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation;

import java.time.Instant;
import java.util.Comparator;

/**
 * Interface for a discrete event in a discrete event simulation
 */
public abstract class DiscreteEvent<EventType> {
    protected Instant time;
    public DiscreteEvent(Instant time){
        this.time = time;
    }
    protected DiscreteEvent(){
        this.time = null;
    }
    public Instant getEventTime(){
        return time;
    }
    public abstract EventType getEventType();

    /**
     *  Used for sorting in ascending order of event time
     */
    static class DiscreteEventComparator implements Comparator<DiscreteEvent<?>>
    {
        public int compare(DiscreteEvent<?> a, DiscreteEvent<?> b)
        {
            return a.getEventTime().compareTo(b.getEventTime());
        }
    }
}
