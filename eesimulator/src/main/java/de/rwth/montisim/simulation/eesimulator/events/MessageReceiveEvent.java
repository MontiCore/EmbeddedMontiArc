/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.message.Message;

/**
 * Event for the completion of a message transfer.
 */
public class MessageReceiveEvent extends EEDiscreteEvent {

	private Message msg;
	/// Might be set if the transmission completion time got invalidated by another
	/// earlier message transfer. (See Bus.)
	public transient boolean invalid;

	public MessageReceiveEvent(Instant eventTime, EEEventProcessor target, Message msg) {
		super(eventTime, target);
		this.msg = msg;
		this.invalid = false;
	}

	protected MessageReceiveEvent() {}

	@Override
	public EEEventType getEventType() {
		return EEEventType.MESSAGE_RECEIVE;
	}

	public Message getMessage() {
		return msg;
	}

	@Typed("receive")
	public static class MessageReceiveEventData extends EventData {
		Message msg;
		MessageReceiveEventData(MessageReceiveEvent event) {
			super(event);
			msg = event.msg;
		}
		@Override
		public EEDiscreteEvent getEvent(ComponentManager cm) {
			return new MessageReceiveEvent(time, EEDiscreteEvent.getTarget(target, cm), msg);
		}
	}
	
	@Override
	public EventData getEventData() {
		return new MessageReceiveEventData(this);
	}
}
