/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.time.Instant;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import org.apache.commons.math3.exception.NullArgumentException;

import simulation.bus.BusMessage;


/**
 * Abstract class that implements all common functionality of EEComponents.
 */
abstract class AbstractEEComponent implements EEComponent{

	private final EESimulator simulator;
	
	private final EEComponentType componentType;
	
	private final UUID Id;
	
	public AbstractEEComponent(EESimulator simulator, EEComponentType type) {
		if (simulator == null || type == null) {
			throw new NullArgumentException();
		}
		this.simulator = simulator;
		this.componentType = type;
		this.Id = UUID.randomUUID();
	}
	
	@Override
	public EEComponentType getComponentType() {
		return componentType;
	}
	
	@Override
	public EESimulator getSimulator() {
		return simulator;
	}

	@Override
	public UUID getId() {
		return Id;
	}
	
	/**
	 * Send message to all targets that are registered for this message in targetsByMessageId
	 * @param message message to be send
	 * @param messageLen length of message in bytes
	 * @param messageId message id of the message
	 * @param eventTime time when the message is sent
	 */
	@Override
	public void sendMessage(Object message, int messageLen , BusEntry messageId, Instant eventTime) {
		List<EEComponent> targets = this.getTargetsByMessageId().getOrDefault(messageId, Collections.emptyList());
		for(EEComponent target : targets) {
			BusMessage msg = new BusMessage(message, 6, messageId,
					eventTime, this.getId(), target);
			this.getSimulator().addEvent(msg);
		}
	}
}
