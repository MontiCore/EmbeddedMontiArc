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
 * Event for the start of a message transfer (e.g. when a component has read a value and starts to (attempt to) send a message )
 */
public class MessageSendEvent extends EEDiscreteEvent {

    public MessageSendEvent(Instant eventTime, EEEventProcessor target, Message msg) {
		super(eventTime, target);
		this.msg = msg;
    }

	@Override
	public EEEventType getEventType(){
		return EEEventType.MESSAGE_SEND;
	}

	private Message msg;

	public Message getMessage(){
		return msg;
	}

}
