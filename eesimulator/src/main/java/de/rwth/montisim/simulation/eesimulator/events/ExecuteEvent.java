/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;

public class ExecuteEvent extends EEEvent {
    public static final int type = registerType(ExecuteEvent.class);
    public final static String TYPE_NAME = "execute";

    public ExecuteEvent(EEComponent target, Instant eventTime) {
        super(target, eventTime);
    }

    protected ExecuteEvent() {
    }

    @Typed(ExecuteEvent.TYPE_NAME)
    public static class ExecuteEventData extends EventData {
        ExecuteEventData(ExecuteEvent event) {
            super(event);
        }

        protected ExecuteEventData() {
        }

        @Override
        public EEEvent getEvent(EESystem eesystem) {
            return new ExecuteEvent(EEEvent.getTarget(target, eesystem), time);
        }
    }

    @Override
    public EventData getEventData() {
        return new ExecuteEventData(this);
    }

    @Override
    public int getType() {
        return type;
    }
}
