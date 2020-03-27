/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.bus.constant;

import java.time.Duration;

import de.rwth.montisim.simulation.eesimulator.*;
import de.rwth.montisim.simulation.eesimulator.bus.*;
import de.rwth.montisim.simulation.eesimulator.events.*;

/**
 * Model for instant transmission of BusMessageEvent.
 */
public class ConstantBus extends Bus {

	private enum TransmissionMode {
		INSTANT, CONSTANT_RATE, CONSTANT_TIME
	}

	/**
	 * INSTANT: Messages directly reach their target when sent. CONSTANT_RATE: Sends
	 * message at the given data rate, ignoring concurrent messages. CONSTANT_TIME:
	 * All messages take "time" seconds to reach their target. Ignores message
	 * concurrency.
	 */
	private TransmissionMode transmissionMode;
	/**
	 * If TransmissionMode.CONSTANT_RATE, contains the transmission rate in
	 * bytes/sec.
	 */
	private double rate;
	/**
	 * If TransmissionMode.CONSTANT_TIME, contains the time to transfer one message.
	 */
	private Duration time;

	private ConstantBus(EESimulator simulator, String name, TransmissionMode mode, double rate, Duration time) {
		super(simulator, name);
		this.transmissionMode = mode;
		this.rate = rate;
		this.time = time;
	}

	/**
	 * Creates a ConstantBus that instantly transmits its message to their targets.
	 * 
	 * @param name The BUS' name.
	 */
	public static ConstantBus newInstantBus(EESimulator simulator, String name) {
		return new ConstantBus(simulator, name, TransmissionMode.INSTANT, 0, null);
	}
	
	/**
	 * Creates a ConstantBus that sends messages with a given data rate (in
	 * bytes/sec) independently of the number of messages sent concurrently.
	 * 
	 * @param name The BUS' name.
	 * @param rate Transmission data rate in Bytes/Second.
	 */
	public static ConstantBus newConstantRateBus(EESimulator simulator, String name, double rate) {
		return new ConstantBus(simulator, name, TransmissionMode.CONSTANT_RATE, rate, null);
	}
	
	/**
	 * Creates a ConstantBus that sends messages with a constant delay independently
	 * of the number of messages sent concurrently.
	 * 
	 * @param name The BUS' name.
	 * @param time Transmission time for any message.
	 */
	public static ConstantBus newConstantTimeBus(EESimulator simulator, String name, Duration time) {
		return new ConstantBus(simulator, name, TransmissionMode.CONSTANT_TIME, 0, time);
	}

	@Override
	protected void sendMessage(MessageSendEvent event) {
		switch(transmissionMode){
		case INSTANT:
			// Directly dispatch the message
			dispatchMessage(new MessageReceiveEvent(event.getEventTime(), null, event.getMessage()));
		break;
		case CONSTANT_RATE:
			double time = event.getMessage().msgLen / rate;
			Duration d = Duration.ofNanos((long)(time*1000000000L));
			simulator.addEvent(new MessageReceiveEvent(event.getEventTime().plus(d), this, event.getMessage()));
		break;
		case CONSTANT_TIME:
			simulator.addEvent(new MessageReceiveEvent(event.getEventTime().plus(this.time), this, event.getMessage()));
		break;
		}
	}

	@Override
	protected void receiveMessage(MessageReceiveEvent event) {
		// Nothing to do, the message is already dispatched
	}

	@Override
	public BusType getBusType() {
		return BusType.INSTANT_BUS;
	}

}
