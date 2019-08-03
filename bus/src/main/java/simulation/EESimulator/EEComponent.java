package simulation.EESimulator;

import java.util.HashMap;
import java.util.List;
import java.util.UUID;

import commons.controller.commons.BusEntry;

public interface EEComponent {
	
	public EEComponentType getComponentType();

	public List<BusEntry> getSubscribedMessages();
	
	public HashMap<BusEntry, List<EEComponent>> getTragetsByMessageId();
	
	public EESimulator getSimulator() ;

	/**
	 * processes an incoming event
	 */
	public void processEvent(EEDiscreteEvent event);

	/**
	 *
	 * @return Randomly generated ID of this component
	 */
	public UUID getId();
}
