/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import simulation.EESimulator.*;

import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public abstract class Bus extends MutableEEComponent {

	private List<EEComponent> connectedComponents = new ArrayList<EEComponent>();

	private Optional<KeepAliveEvent> keepAlive = Optional.empty();

	protected Bus(EESimulator simulator) {
		super(simulator, EEComponentType.BUS);
	}

	public void processEvent(EEDiscreteEvent evt) {
		if (evt.getEventTime().isBefore(this.getCurrentTime())) {
			String msgId = " ";
			if(evt.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE){
				msgId = " " +((BusMessage)evt).getMessageID() + " ";
			}
			throw new IllegalArgumentException("Can not process event:" +msgId+ "Time of event already simulated! Last time was " + this.getCurrentTime() + " Event time is: " + evt.getEventTime());
		}
		if (evt.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) {
			BusMessage msg = (BusMessage) evt;
			// only work on messages if it was not already seen before
			if (!msg.hasTraveresed(this)) {
				this.simulateUntil(msg.getEventTime());
				this.registerMessage(msg);
				this.setKeepAlive();
			}
		} else if (evt.getEventType() == EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT) {
			if (this.keepAlive.get().getId() == evt.getId()) {
				this.simulateUntil(evt.getEventTime());
				if (this.hasMessages()) {
					this.setKeepAlive();
				} else {
					this.keepAlive = Optional.empty();
				}
			}
		} else {
			throw new IllegalArgumentException(
					"Invalid event type. Expected KeepAliveEvent or BusMessage but was " + evt.getEventType());
		}

	}

	public void registerComponent(EEComponent component) {
		if (connectedComponents.contains(component)) {
			throw new IllegalArgumentException(
					"Component" + component.toString() + "is already registered at " + this.toString() + ".");
		} else {
			connectedComponents.add(component);
			addSubscribedMessages(component, component.getSubscribedMessages());
		}

	}

	public void addSubscribedMessages(EEComponent component, List<BusEntry> messages) {
		List<BusEntry> newMessages = new ArrayList<BusEntry>();
		for (BusEntry message : messages) {
			List<EEComponent> targets = targetsByMessageId.getOrDefault(message, new ArrayList<EEComponent>());
			if(!targets.contains(component)) {
				targets.add(component);
				targetsByMessageId.put(message, targets);
				newMessages.add(message);
			}
			if (!subscribedMessages.contains(message)) {
				subscribedMessages.add(message);
			}
		}
		if(!newMessages.isEmpty()) {
			for (EEComponent connectedComponent : connectedComponents) {
				if (connectedComponent != component && connectedComponent.getComponentType() == EEComponentType.BRIDGE) {
					((Bridge) connectedComponent).update(this, newMessages);
				}
			}
		}

	}

	public List<EEComponent> getConnectedComponents() {
		return this.connectedComponents;
	}

	public abstract Instant getCurrentTime();

	protected void registerMessageAtSimulator(BusMessage msg) {
		for (EEComponent target : this.targetsByMessageId.getOrDefault(msg.getMessageID(), Collections.emptyList())) {
			this.getSimulator().addEvent(msg.forwardTo(target));
		}
	}

	protected void setKeepAlive() {
		Instant nextFinishTime = this.getNextFinishTime();
		// at least one message present AND (old keepAlive empty OR old keepAlive
		// already processed OR new keepAlive
		// before old one)
		if (this.hasMessages()
				&& (!this.keepAlive.isPresent() || !keepAlive.get().getEventTime().isAfter(this.getCurrentTime())
						|| nextFinishTime.isBefore(keepAlive.get().getEventTime()))) {
			keepAlive = Optional.of(new KeepAliveEvent(nextFinishTime, this));
			this.getSimulator().addEvent(keepAlive.get());
			System.out.println("Bus: " + this.getId() + " has set new keepAliveEvent with time: " + nextFinishTime);
		}
	}

	protected long calculateTransmissionTime(long transmittedBytes) {
		if(transmittedBytes < 0){
			throw new IllegalArgumentException("Transmitted bytes must be zero or positive. Transmitted bytes was: " + transmittedBytes);
		}
		long transmittedMikroBits = transmittedBytes * 8000000L;
		return (long) Math.ceil(transmittedMikroBits / this.getOperationMode().getDataRate());
	}

	protected abstract void simulateUntil(Instant endTime);

	protected abstract Instant getNextFinishTime();

	protected abstract void registerMessage(BusMessage msg);

	protected abstract boolean hasMessages();

	protected abstract OperationMode getOperationMode();

	public abstract BusType  getBusType();

}
