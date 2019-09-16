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


public abstract class ImmutableEEComponent extends AbstractEEComponent {
	private final HashMap<BusEntry, List<EEComponent>> targetsByMessageId; // which message to which component
	private final List<BusEntry> subscribedMessages; // which message do I want to get

	public ImmutableEEComponent(EESimulator simulator, EEComponentType type, List<BusEntry> subscribedMessages,
			HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
		super(simulator, type);
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
