/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.PortInformation.PortDirection;
import de.rwth.montisim.commons.eventsimulation.DiscreteEvent;
import de.rwth.montisim.commons.eventsimulation.exceptions.UnexpectedEventException;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;

/**
 * Components that receives and sends messages in a car. The component must
 * register its different ports.
 */
public abstract class EEComponent extends BusUser {
	public EEComponent(BusUserProperties properties) {
		super(properties);
	}

	transient List<PortInformation> inputPorts = new ArrayList<>();
	transient List<PortInformation> outputPorts = new ArrayList<>();

	/**
	 * Returns the messages that this component wants to receive.
	 */
	public List<PortInformation> getInputPorts() {
		return inputPorts;
	}

	/**
	 * Returns the messages that this component sends.
	 */
	public List<PortInformation> getOutputPorts() {
		return outputPorts;
	}

	public MessageInformation addInput(String name, DataType type, boolean multipleInputsAllowed, boolean optional)
			throws EEMessageTypeException {
		inputPorts.add(new PortInformation(name, type, PortDirection.INPUT, multipleInputsAllowed, optional));
		return eesystem.getMessageTypeManager().registerMessage(name, type, this);
	}

	public MessageInformation addInput(String name, DataType type, boolean multipleInputsAllowed)
			throws EEMessageTypeException {
		return addInput(name, type, multipleInputsAllowed, false);
	}

	public MessageInformation addInput(String name, DataType type) throws EEMessageTypeException {
		return addInput(name, type, false, false);
	}

	public MessageInformation addOptionalInput(String name, DataType type, boolean multipleInputsAllowed)
			throws EEMessageTypeException {
		return addInput(name, type, multipleInputsAllowed, true);
	}

	public MessageInformation addOutput(String name, DataType type) throws EEMessageTypeException {
		outputPorts.add(new PortInformation(name, type, PortDirection.OUTPUT, false, true));
		return eesystem.getMessageTypeManager().registerMessage(name, type, this);
	}


	@Override
    public void process(DiscreteEvent event) {
		int type = event.getType();
		if (type == MessageSendEvent.type){
			dispatchMessage((MessageSendEvent) event);
		} else if (type == MessageReceiveEvent.type){
			receive((MessageReceiveEvent) event);
		} else throw new UnexpectedEventException(this.toString(), event);
	}
	
	public void sendMessage(Instant time, MessageInformation info, Object message, int msgLen) {
		this.eesystem.simulator.addEvent(new MessageSendEvent(this, time, new Message(this, info, message, msgLen)));
	}

	public void sendMessage(Instant time, MessageInformation info, Object message) {
		this.eesystem.simulator.addEvent(new MessageSendEvent(this, time, new Message(this, info, message)));
	}

	protected abstract void receive(MessageReceiveEvent msgRecvEvent);
}
