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


public interface EEComponent {
	
	public EEComponentType getComponentType();

	public List<BusEntry> getSubscribedMessages();
	
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
	
	
	public void sendMessage(Object message, int messageLen , BusEntry messageId, Instant eventTime);
}
