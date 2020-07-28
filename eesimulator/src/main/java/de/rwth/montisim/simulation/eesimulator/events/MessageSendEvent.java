/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.message.Message;

/**
 * Event for the start of a message transfer (e.g. when a component has read a
 * value and starts to (attempt to) send a message )
 */
public class MessageSendEvent extends EEDiscreteEvent {

	private Message msg;

	public MessageSendEvent(Instant eventTime, EEEventProcessor target, Message msg) {
		super(eventTime, target);
		this.msg = msg;
	}

	protected MessageSendEvent() {
	}

	@Override
	public EEEventType getEventType() {
		return EEEventType.MESSAGE_SEND;
	}

	public Message getMessage() {
		return msg;
	}

	@Typed("send")
	public static class MessageSendEventData extends EventData {
		Message msg;
		MessageSendEventData(MessageSendEvent event) {
			super(event);
			msg = event.msg;
		}
        protected MessageSendEventData() {}
		@Override
		public EEDiscreteEvent getEvent(ComponentManager cm) {
			return new MessageReceiveEvent(time, EEDiscreteEvent.getTarget(target, cm), msg);
		}
	}
	
	@Override
	public EventData getEventData() {
		return new MessageSendEventData(this);
	}

}
