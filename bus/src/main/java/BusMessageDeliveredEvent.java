import commons.simulation.DiscreteEvent;

public class BusMessageDeliveredEvent implements DiscreteEvent{

	private BusMessage message;
	
	public BusMessageDeliveredEvent(BusMessage msg) {
		this.message = msg;
	}
	
	public BusMessage getMessage() {
		return this.message;
	}
	
	@Override
	public long getEventTime() {
		return this.message.getFinishTime();
	}


	@Override
	public int getEventId() {
		return this.message.getMessageID().ordinal();
	}
}
