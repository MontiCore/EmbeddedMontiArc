import java.time.Instant;
import java.util.UUID;

import commons.simulation.DiscreteEvent;

public class KeepAliveEvent implements DiscreteEvent{
	
	private String ID;
	
	private Instant eventTime;
	
	public KeepAliveEvent(Instant eventTime) {
		this.eventTime = eventTime;
		this.ID = UUID.randomUUID().toString();
	}
	
	
	@Override
	public Instant getEventTime() {
		return eventTime;
	}

	@Override
	public String getEventId() {
		return ID;
	}

}
