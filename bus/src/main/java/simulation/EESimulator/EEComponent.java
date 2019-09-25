/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;

import java.time.Instant;
import java.util.HashMap;
import java.util.List;
import java.util.UUID;

/**
 * Components that transmit, receives, sends messages in a car.
 */
public interface EEComponent {


	public EEComponentType getComponentType();

	/**
	 * Return the messages that this component wants to receive.
	 * @return
	 */
	public List<BusEntry> getSubscribedMessages();

	/**
	 * Return a map with registered components for each message
	 * @return Map with registered components for each message
	 */
	public HashMap<BusEntry, List<EEComponent>> getTargetsByMessageId();
	
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


	/**
	 * Send a message to all targets registered at this component.
	 * @param message Message to be sent
	 * @param messageLen Length of the message
	 * @param messageId Id of the message
	 * @param eventTime Time when the message is send
	 */
	public void sendMessage(Object message, int messageLen , BusEntry messageId, Instant eventTime);
}
