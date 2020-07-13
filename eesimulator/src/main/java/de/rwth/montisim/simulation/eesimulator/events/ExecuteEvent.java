/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;

public class ExecuteEvent extends EEDiscreteEvent {
	public final static String TYPE = "execute";

	public ExecuteEvent(Instant eventTime, EEEventProcessor target) {
		super(eventTime, target);
	}

	protected ExecuteEvent() {
	}

	@Override
	public EEEventType getEventType() {
		return EEEventType.EXECUTE;
	}

	@Typed("execute")
	public static class ExecuteEventData extends EventData {
		ExecuteEventData(ExecuteEvent event) {
			super(event);
		}
		@Override
		public EEDiscreteEvent getEvent(ComponentManager cm) {
			return new ExecuteEvent(time, EEDiscreteEvent.getTarget(target, cm));
		}
	}
	
	@Override
	public EventData getEventData() {
		return new ExecuteEventData(this);
	}
}
