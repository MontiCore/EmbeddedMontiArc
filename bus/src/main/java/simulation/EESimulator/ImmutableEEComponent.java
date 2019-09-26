/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;

import java.util.HashMap;
import java.util.List;

/**
 * EEComponent that knows its targets upon creating the component
 */
public abstract class ImmutableEEComponent extends AbstractEEComponent {

	/**
	 * Targets indexed by message ids. (To which target does this component send which message)
	 */
	private final HashMap<BusEntry, List<EEComponent>> targetsByMessageId;

	/**
	 * Messages that this component listens to.
	 */
	private final List<BusEntry> subscribedMessages;

	public ImmutableEEComponent(EESimulator simulator, EEComponentType type, List<BusEntry> subscribedMessages,
			HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
		super(simulator, type);
		if(subscribedMessages == null || targetsByMessageId == null) {
			throw new IllegalArgumentException("subscribedMessages and targetsByMessageId can not be null");
		}
		this.subscribedMessages = subscribedMessages;
		this.targetsByMessageId = targetsByMessageId;
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
