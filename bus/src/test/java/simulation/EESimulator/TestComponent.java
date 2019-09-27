/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import simulation.bus.BusMessage;

public class TestComponent extends ImmutableEEComponent {
	
	private Set<Object> processedMessages = new HashSet<Object>();

	public TestComponent(EESimulator simulator) {
		super(simulator, EEComponentType.TEST_COMPONENT, Arrays.asList(BusEntry.values()), new HashMap<BusEntry, List<EEComponent>>());
	}
    public TestComponent(EESimulator simulator, List<BusEntry> listenTo) {
        super(simulator, EEComponentType.TEST_COMPONENT, listenTo, new HashMap<BusEntry, List<EEComponent>>());
    }

	@Override
	public void processEvent(EEDiscreteEvent event) {
		if(event.getEventType() != EEDiscreteEventTypeEnum.BUSMESSAGE) {
			throw new IllegalArgumentException("Event has to be a bus message but was " + event.getEventType());
		}
		else {
			BusMessage message = (BusMessage)event;
			if(!this.getSubscribedMessages().contains(message.getMessageID())) {
				throw new IllegalArgumentException("Message has unexpected messageId");
			}
			else{
				processedMessages.add(message.getMessage());
			}
		}
	}
	
	public Set<Object> getProcessedMessages(){
		return this.processedMessages;
	}
	
	public boolean processedMessage(Object message) {
		return this.processedMessages.contains(message);
	}

}
