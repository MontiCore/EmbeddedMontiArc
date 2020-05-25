/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.simulation.eesimulator.components.ComponentManager;
import de.rwth.montisim.simulation.eesimulator.events.EEDiscreteEvent;
import de.rwth.montisim.simulation.eesimulator.events.EEEventType;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupErrors;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessagePriorityComparator;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;

/**
 * Discrete event simulator that takes care of the communication in the car.
 * 
 * EE Setup Process: - Create an EESimulator. - Create buses -> Give
 * EESimulator. - Create Components/Bridges -> Give EESimulator. -
 * connectToBus(id/name/bus) for components/bridges. - finalizeSetup()
 * 
 * For more information on the EE-system, see [docs/eesimulator.md]
 */
public class EESimulator extends DiscreteEventSimulator<EEEventType, EEDiscreteEvent> {

	// Vector<DataType> types = new Vector<>();
	// HashMap<DataType, Integer> typeIds = new HashMap<>();

    protected final MessagePriorityComparator msgPrioComp;
	protected final ComponentManager componentManager;
	protected final MessageTypeManager messageTypeManager;
	protected final EESetupErrors errors;
	private boolean finalized = false;

	public EESimulator(MessageTypeManager messageTypeManager) {
		this.errors = new EESetupErrors();
		this.componentManager = new ComponentManager(this.errors);
		this.messageTypeManager = messageTypeManager;
        this.msgPrioComp = new MessagePriorityComparator(messageTypeManager, componentManager);
	}

	public void processEvent(EEDiscreteEvent event) {
		event.getTarget().process(event);
	}

	public MessageTypeManager getMessageTypeManager() {
		return messageTypeManager;
	}

	public ComponentManager getComponentManager() {
		return componentManager;
	}

	public MessagePriorityComparator getMsgPrioComp() {
		return msgPrioComp;
	}

	public void finalizeSetup() throws EESetupException {
		componentManager.finalizeSetup();
		errors.throwExceptions();
		this.finalized = true;
	}

	public boolean isFinalized(){
		return finalized;
	}
}
