import commons.simulation.DiscreteEvent;

public class SimulationEndEvent implements DiscreteEvent {

	long endTime;
	
	public SimulationEndEvent(long endTime) {
		this.endTime = endTime;
	}
	
	@Override
	public long getEventTime() {
		return this.endTime;
	}

	@Override
	public int getEventId() {
		// TODO Auto-generated method stub
		return 0;
	}

}
