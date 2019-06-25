

import commons.simulation.DiscreteEvent;

public class TestComponent extends EEComponent {
	
	private String ID;

	public TestComponent(EESimulator simulator, String ID) {
		super(simulator);
		this.ID = ID;
	}

	@Override
	public void processEvent(DiscreteEvent event) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public String getID() {
		return "TestComponent " + ID;
	}

}
