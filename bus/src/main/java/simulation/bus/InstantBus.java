/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

import java.time.Instant;

import simulation.EESimulator.EESimulator;

public class InstantBus extends Bus {

	private Instant currentTime = Instant.EPOCH;
	
	public InstantBus(EESimulator sim) {
		super(sim);
	}

	@Override
	public Instant getCurrentTime() {
		return currentTime;
	}

	@Override
	protected void simulateUntil(Instant endTime) {
		currentTime = endTime;
	}

	@Override
	protected Instant getNextFinishTime() {
		return currentTime;
	}

	@Override
	protected void registerMessage(BusMessage msg) {
		msg.transmitBytes(msg.getRemainingBytes(), 0.0);
		msg.setFinishTime(msg.getEventTime());
		this.registerMessageAtSimulator(msg);
	}

	@Override
	protected boolean hasMessages() {
		return false;
	}

	@Override
	public BusType getBusType() {
		return BusType.INSTANT_BUS;
	}
}
