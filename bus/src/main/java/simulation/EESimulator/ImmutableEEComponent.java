package simulation.EESimulator;

import java.util.HashMap;
import java.util.List;

import commons.controller.commons.BusEntry;

public abstract class ImmutableEEComponent extends AbstractEEComponent {
	private final HashMap<BusEntry, List<EEComponent>> targetsByMessageId; // which message to which component
	private final List<BusEntry> subscribedMessages; // which message do I want to get

	public ImmutableEEComponent(EESimulator simulator, EEComponentType type, List<BusEntry> listenTo,
			HashMap<BusEntry, List<EEComponent>> targetsByMessageId) {
		super(simulator, type);
		this.subscribedMessages = listenTo;
		this.targetsByMessageId = targetsByMessageId;
	}

	@Override
	public List<BusEntry> getSubscribedMessages() {
		return subscribedMessages;
	}

	@Override
	public HashMap<BusEntry, List<EEComponent>> getTragetsByMessageId() {
		return targetsByMessageId;
	}

}
