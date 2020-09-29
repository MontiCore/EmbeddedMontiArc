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
public class MessageSendEvent extends EEEvent {
	public static final int type = registerType(MessageSendEvent.class);
	public final static String TYPE_NAME = "send";

	private Message msg;

	public MessageSendEvent(EEEventProcessor target, Instant eventTime, Message msg) {
		super(target, eventTime);
		this.msg = msg;
	}

	protected MessageSendEvent() {
	}

	public Message getMessage() {
		return msg;
	}

	@Typed(MessageSendEvent.TYPE_NAME)
	public static class MessageSendEventData extends EventData {
		Message msg;
		MessageSendEventData(MessageSendEvent event) {
			super(event);
			msg = event.msg;
		}
        protected MessageSendEventData() {}
		@Override
		public EEEvent getEvent(ComponentManager cm) {
			return new MessageReceiveEvent(EEEvent.getTarget(target, cm), time, msg);
		}
	}
	
	@Override
	public EventData getEventData() {
		return new MessageSendEventData(this);
	}

	@Override
	public int getType() {
		return type;
	}

}
