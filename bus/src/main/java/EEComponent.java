import commons.controller.commons.BusEntry;
import commons.simulation.DiscreteEvent;

public abstract class EEComponent {
	
	protected EESimulator simulator;
	
	public EEComponent(EESimulator simulator) {
		this.simulator = simulator;
	}

    /**
     * processes an incoming event
     */
    public abstract void processEvent(DiscreteEvent event);

    /**
     *
     * @return Id of this component. Can be a BusEntry or randomly generated ID
     */
    public abstract String getID();
}
