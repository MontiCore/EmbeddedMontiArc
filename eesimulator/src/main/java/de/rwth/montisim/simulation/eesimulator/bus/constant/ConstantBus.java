/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.constant;

import java.time.Duration;

import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.simulation.eesimulator.bus.*;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties.BusType;
import de.rwth.montisim.simulation.eesimulator.events.*;

/**
 * Model for instant transmission of BusMessageEvent.
 */
public class ConstantBus extends Bus {
	public final ConstantBusProperties properties;

	public ConstantBus(ConstantBusProperties properties) {
		super(properties);
		this.properties = properties;
	}

	

	@Override
	protected void sendMessage(MessageSendEvent event) {
		switch(properties.mode){
		case INSTANT:
			// Directly dispatch the message
			dispatchMessage(new MessageReceiveEvent(event.getEventTime(), null, event.getMessage()));
		break;
		case CONSTANT_RATE:
			double time = event.getMessage().msgLen / properties.rate;
			Duration d = Time.durationFromSeconds(time);
			simulator.addEvent(new MessageReceiveEvent(event.getEventTime().plus(d), this, event.getMessage()));
		break;
		case CONSTANT_TIME:
			simulator.addEvent(new MessageReceiveEvent(event.getEventTime().plus(properties.time), this, event.getMessage()));
		break;
		}
	}

	@Override
	protected void receiveMessage(MessageReceiveEvent event) {
		// Nothing to do, the message is already dispatched
	}

	@Override
	public BusType getBusType() {
		return BusType.CONSTANT_BUS;
	}

	@Override
	protected void init() {

	}

}
