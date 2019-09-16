/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.util.HashMap;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import org.apache.commons.math3.exception.NullArgumentException;


import java.util.ArrayList;
import java.util.List;

public abstract class MutableEEComponent extends AbstractEEComponent{

	protected HashMap<BusEntry, List<EEComponent>> targetsByMessageId; // which message to which component
	protected List<BusEntry> subscribedMessages; // which message do I want to get

	protected MutableEEComponent(EESimulator simulator, EEComponentType type) {
		super(simulator, type);
		if (simulator == null || type == null) {
			throw new NullArgumentException();
		}
		targetsByMessageId = new HashMap<BusEntry, List<EEComponent>>();
		subscribedMessages = new ArrayList<BusEntry>();
	}
	
	@Override
	public List<BusEntry> getSubscribedMessages() {
		return subscribedMessages;
	}
	
	@Override
	public HashMap<BusEntry, List<EEComponent>> getTargetsByMessageId() {
		return targetsByMessageId;
	}
}
