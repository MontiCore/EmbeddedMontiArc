/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.events;

import java.time.Instant;

import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.message.Message;

/**
 * Event for the completion of a message transfer.
 */
public class MessageReceiveEvent extends EEDiscreteEvent {

	private Message msg;
	/// Might be set if the transmission completion time got invalidated by another earlier message transfer. (See Bus.)
	public boolean invalid;

    public MessageReceiveEvent(Instant eventTime, EEEventProcessor target, Message msg) {
		super(eventTime, target);
		this.msg = msg;
		this.invalid = false;
    }

	@Override
	public EEEventType getEventType(){
		return EEEventType.MESSAGE_RECEIVE;
	}

	public Message getMessage(){
		return msg;
	}

}
