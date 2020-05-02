/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.message.PortInformation;
import de.rwth.montisim.simulation.eesimulator.message.PortInformation.PortDirection;

/**
 * Components that receives and sends messages in a car. The component must
 * register its different ports.
 */
public abstract class EEComponent extends BusComponent {
    List<PortInformation> inputPorts = new ArrayList<>();
    List<PortInformation> outputPorts = new ArrayList<>();

	public EEComponent(EESimulator simulator, String name, int priority) {
		super(simulator, name, priority);
	}
	public EEComponent(EESimulator simulator, String name) {
		this(simulator, name, 0);
	}


	/**
	 * Returns the messages that this component wants to receive.
	 */
	public List<PortInformation> getInputPorts(){
		return inputPorts;
	}
	/**
	 * Returns the messages that this component sends.
	 */
	public List<PortInformation> getOutputPorts(){
		return outputPorts;
	}

	public MessageInformation addInput(String name, DataType type, boolean multipleInputsAllowed, boolean optional) {
		MessageInformation m = new MessageInformation(name, type, simulator.messageTypeManager, this);
		inputPorts.add(new PortInformation(m, PortDirection.INPUT, multipleInputsAllowed, optional));
		return m;
	}

	public MessageInformation addInput(String name, DataType type, boolean multipleInputsAllowed) {
		return addInput(name, type, multipleInputsAllowed, false);
	}
	public MessageInformation addInput(String name, DataType type) {
		return addInput(name, type, false, false);
	}

	public MessageInformation addOptionalInput(String name, DataType type, boolean multipleInputsAllowed) {
		return addInput(name, type, multipleInputsAllowed, true);
	}

	public MessageInformation addOutput(String name, DataType type) {
		MessageInformation m = new MessageInformation(name, type, simulator.messageTypeManager, this);
		outputPorts.add(new PortInformation(m, PortDirection.OUTPUT, false, true));
		return m;
	}


	@Override
    public void process(EEDiscreteEvent event) {
		switch(event.getEventType()){
			case MESSAGE_SEND:
				dispatchMessage((MessageSendEvent) event);
			break;
			case MESSAGE_RECEIVE:
                receive((MessageReceiveEvent) event);
			break;
			default:
				throw new UnexpectedEventException(this.toString(), event);
		}
	}
	
	public void send(Instant time, Message msg) {
		this.simulator.addEvent(new MessageSendEvent(time, this, msg));
	}

	protected abstract void receive(MessageReceiveEvent msgRecvEvent);
}
