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
public class MessageReceiveEvent extends EEEvent {
	public static final int type = registerType(MessageReceiveEvent.class);
	public final static String TYPE_NAME = "receive";

	private Message msg;
	/// Might be set if the transmission completion time got invalidated by another
	/// earlier message transfer. (See Bus.)
	public transient boolean invalid;

	public MessageReceiveEvent(EEEventProcessor target, Instant eventTime, Message msg) {
		super(target, eventTime);
		this.msg = msg;
		this.invalid = false;
	}

	protected MessageReceiveEvent() {}

	public Message getMessage() {
		return msg;
	}

	@Typed(MessageReceiveEvent.TYPE_NAME)
	public static class MessageReceiveEventData extends EventData {
		Message msg;
		MessageReceiveEventData(MessageReceiveEvent event) {
			super(event);
			msg = event.msg;
		}
        protected MessageReceiveEventData() {}
		@Override
		public EEEvent getEvent(ComponentManager cm) {
			return new MessageReceiveEvent(EEEvent.getTarget(target, cm), time, msg);
		}
	}
	
	@Override
	public EventData getEventData() {
		return new MessageReceiveEventData(this);
	}

	@Override
	public int getType() {
		return type;
	}
}
