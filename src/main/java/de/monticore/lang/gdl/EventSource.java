package de.monticore.lang.gdl;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;

public class EventSource<E, D> {
    
    private Map<E, List<Consumer<D>>> observers = new HashMap<>();

    /**
     * Add an observer to listen to all events.
     * @param observer Observer to add.
     * @return true, if added successfully, false otherwise
     */
    public boolean addObserver(E event, Consumer<D> observer) {
        if (observers.get(event) == null) {
            observers.put(event, new ArrayList<>());
        }
        return observers.get(event).add(observer);
    }

    /**
     * Remove an observer from the observer list.
     * @param observer Observer to remove.
     * @return true, if removed successfully, false otherwise
     */
    public boolean removeObserver(E event, Consumer<D> observer) {
        if (observers.get(event) == null) {
            return false;
        }
        return observers.get(event).remove(observer);
    }

    /**
     * Remove all observers
     */
    public void removeAll() {
        for (E event: observers.keySet()) {
            observers.remove(event);
        }
    }

    /**
     * Notify all observers of an event.
     * @param data 
     */
    protected void notifyAll(E event, D data) {
        if (observers.get(event) == null) return;
        observers.get(event).stream().forEach(o -> o.accept(data));
    }

}
