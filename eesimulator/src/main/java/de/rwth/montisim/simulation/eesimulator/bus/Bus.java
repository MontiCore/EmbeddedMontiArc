/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus;

import de.rwth.montisim.commons.eventsimulation.exceptions.*;
import de.rwth.montisim.simulation.eesimulator.bus.BusProperties.BusType;
import de.rwth.montisim.simulation.eesimulator.components.BusUser;
import de.rwth.montisim.simulation.eesimulator.components.EEComponentType;
import de.rwth.montisim.simulation.eesimulator.components.EEEventProcessor;
import de.rwth.montisim.simulation.eesimulator.events.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Buses are used to transmit messages in the vehicle between EEComponents (and
 * through Bridges). Current implementations: ConstantBus, CAN & FlexRay.
 */
public abstract class Bus extends EEEventProcessor {

	// public static Bus buildBus(BusProperties properties, MessagePriorityComparator comp){
	// 	switch(properties.getBusType()) {
	// 		case CAN:
	// 			return new CAN((CANProperties)properties, comp);
	// 		case CONSTANT_BUS:
	// 			return new ConstantBus((ConstantBusProperties)properties);
	// 		case FLEXRAY:
	// 		 	// TODO
	// 			return null;
	// 		default:
	// 			return null;
	// 	}
	// }

	/**
	 * Components that are connected to this bus.
	 */
	protected final transient List<BusUser> connectedComponents = new ArrayList<>();

	protected final transient HashMap<Integer, List<BusUser>> msgTargets = new HashMap<>();

	protected Bus(BusProperties properties) {
		super(properties);
	}

	public EEComponentType getComponentType(){
		return EEComponentType.BUS;
	}

	/**
	 * Accepts MessageSendEvent and MessageReceiveEvent events.
	 * MessageSendEvent: When a component connected to the bus starts to (attempt to) transmit a message.
	 * MessageReceiveEvent: When a message transmission is completed (message gets passed to its targets).
	 * 						This Event is only created by the BUS itself.
	 */
	public void process(EEDiscreteEvent evt) {
		switch(evt.getEventType()){
			case MESSAGE_SEND:
				sendMessage((MessageSendEvent) evt);
			break;
			case MESSAGE_RECEIVE:
				MessageReceiveEvent msgRecvEvent = (MessageReceiveEvent) evt;
				if (!msgRecvEvent.invalid){
					dispatchMessage(msgRecvEvent);
					receiveMessage(msgRecvEvent);
				}
			break;
			default:
				throw new UnexpectedEventException(this.toString(), evt);
		}
	}

	/** Dispatches the given Message to all its targets, effectively completing its transmission in this BUS. */
	protected void dispatchMessage(MessageReceiveEvent msgRecvEvent) {
		List<BusUser> targets = msgTargets.get(msgRecvEvent.getMessage().msgId);
		if (targets == null) throw new IllegalArgumentException("Tried to dispatch a message with no associated targets. (event: "+msgRecvEvent+").");
		for(BusUser e : targets){
			e.process(msgRecvEvent);
		}
	}

	public void addComponent(BusUser component) {
		if (connectedComponents.contains(component))
			throw new IllegalArgumentException("Component " + component + " is already registered at " + this + ".");
		connectedComponents.add(component);
	}

	public void addMessageTargets(int msgId, List<BusUser> targets){
		if (msgTargets.containsKey(msgId)) throw new IllegalArgumentException("Targets already registered for msgId: " + msgId);
		msgTargets.put(msgId, targets);
	}

	public List<BusUser> getConnectedComponents() {
		return this.connectedComponents;
	}

	/**
	 * Set a event that is register to the bus itself with event time of the next expected finish time of the next message.
	 */
	// protected void setKeepAlive() {
	// 	Instant nextFinishTime = this.getNextFinishTime();
	// 	// at least one message present AND (old keepAlive empty OR old keepAlive
	// 	// already processed OR new keepAlive
	// 	// before old one)
	// 	if (this.hasMessages()
	// 			&& (!this.keepAlive.isPresent() || !keepAlive.get().getEventTime().isAfter(this.currentTime)
	// 					|| nextFinishTime.isBefore(keepAlive.get().getEventTime()))) {
	// 		keepAlive = Optional.of(new KeepAliveEvent(nextFinishTime, this));
	// 		this.getSimulator().addEvent(keepAlive.get());
	// 	}
	// }

	/**
	 * Calculate the amount of time that is necessary to transmit transmittedBytes.
	 * @param transmittedBytes Bytes for which the transmission time should be calculated.
	 * @return Amount of time necessary to transmit transmittedBytes in nanoseconds.
	 */
	// TODO
	// protected long calculateTransmissionTime(long transmittedBytes) {
	// 	if(transmittedBytes < 0){
	// 		throw new IllegalArgumentException("Transmitted bytes must be zero or positive. Transmitted bytes was: " + transmittedBytes);
	// 	}
	// 	long transmittedMicroBits = transmittedBytes * 8000000L;
	// 	return (long) Math.ceil(transmittedMicroBits / this.getOperationMode().getDataRate());
	// }

	/**
	 * Process a MessageSendEvent: register the message for transmission and handle current transmissions up to the event time.
	 * If the new message changes the current next MessageReceiveEvent, it has to be invalidated.
	 * Responsible of registering the next MessageReceiveEvent for the next message transmission completion.
	 */
	protected abstract void sendMessage(MessageSendEvent event);
	
	/**
	 * Update the bus simulation to when the event's message is transmitted.
	 * The message itself is already checked for validity and dispatched to its targets.
	 * Responsible of registering the next MessageReceiveEvent if there are still messages to be transmitted.
	 */
	protected abstract void receiveMessage(MessageReceiveEvent event);

	public abstract BusType getBusType();

	@Override
	public String toString(){
		return getBusType() + " bus \"" + properties.name+'"';
	}

	public HashMap<Integer, List<BusUser>> getMsgTargets(){
		return msgTargets;
	}
}
