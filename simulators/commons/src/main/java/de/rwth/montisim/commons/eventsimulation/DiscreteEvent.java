/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.eventsimulation;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;

/**
 * Interface for a discrete event in a discrete event simulation
 */
public abstract class DiscreteEvent {
    protected EventTarget target;
    protected Instant time;
	/// Might be set if the another event invalidated this one -> is then filtered out
	public transient boolean invalid;
    public DiscreteEvent(EventTarget target, Instant time){
        this.target = target;
        this.time = time;
        this.invalid = false;
    }
    protected DiscreteEvent(){
    }
    public Instant getEventTime(){
        return time;
    }
    public EventTarget getTarget(){
        return target;
    }
    public abstract int getType();
    /**
     *  Used for sorting in ascending order of event time
     */
    static class DiscreteEventComparator implements Comparator<DiscreteEvent>
    {
        public int compare(DiscreteEvent a, DiscreteEvent b)
        {
            return a.getEventTime().compareTo(b.getEventTime());
        }
    }

    // Event classes register their type here
    protected synchronized static int registerType(Class<?> c){
        if (typeMap.containsKey(c)) return typeMap.get(c);
        int type = typeCounter;
        ++typeCounter;
        typeMap.put(c, type);
        types.add(c);
        return type;
    }
    public static String getTypeName(int type){
        return types.get(type).getSimpleName();
    }
    static List<Class<?>> types = new ArrayList<>();
    static HashMap<Class<?>, Integer> typeMap = new HashMap<>();
    static int typeCounter = 0;
}
