/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.utils.ParsingException;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;

import java.time.Instant;
import java.util.Optional;

/**
 * Discrete Events that are processed by the EESimulator.
 */
public abstract class EEEvent extends DiscreteEvent {

    public EEEvent(EEComponent target, Instant eventTime) {
        super(target, eventTime);
    }

    protected EEEvent() {
    }

    @Override
    public String toString() {
        return "EEEvent {type: " + getClass().getSimpleName() + ", target: " + this.target + ", time: " + this.time + "}";
    }

    public static abstract class EventData {
        Instant time;
        String target;

        EventData(EEEvent event) {
            this.time = event.time;
            this.target = ((EEComponent) event.target).properties.name;
        }

        protected EventData() {
        }

        public abstract EEEvent getEvent(EESystem eesystem);
    }

    public abstract EventData getEventData();

    protected static EEComponent getTarget(String name, EESystem eesystem) {
        Optional<EEComponent> r = eesystem.getComponent(name);
        if (!r.isPresent()) throw new ParsingException("Unknown target component: " + name);
        return r.get();
    }
}