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
		msg.setFinishTime(currentTime);
		this.registerMessageAtSimulator(msg);
	}

	@Override
	protected boolean hasMessages() {
		return false;
	}

}
