import commons.simulation.DiscreteEvent;

public class BusMessageTransmissionRequestEvent implements DiscreteEvent{

	private BusMessage message;
	
	public BusMessageTransmissionRequestEvent(BusMessage msg) {
		this.message = msg;
	}
	
	public BusMessage getMessage() {
		return this.message;
	}
	
	
	@Override
	public long getEventTime() {
		return this.message.getRequestTime();
	}


	@Override
	public int getEventId() {
		return this.message.getMessageID().ordinal();
	}
}