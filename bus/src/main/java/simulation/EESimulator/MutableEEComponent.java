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

/**
 * EEComponent that does not know its targets upon creating the component.
 */
public abstract class MutableEEComponent extends AbstractEEComponent{

	/**
	 * Targets indexed by message ids. (To which target does this component send which message)
	 */
	protected final HashMap<BusEntry, List<EEComponent>> targetsByMessageId;

	/**
	 * Messages that this component listens to.
	 */
	protected final List<BusEntry> subscribedMessages;

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
