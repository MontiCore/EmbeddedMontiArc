/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

import java.time.Instant;
import java.util.Comparator;
import java.util.HashSet;
import java.util.Random;
import java.util.Set;
import java.util.UUID;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.controller.commons.BusEntry;
import org.apache.commons.math3.exception.NullArgumentException;

import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EEDiscreteEventTypeEnum;

/**
 * Messages that are transmitted over buses between EEComponent(e.g. Sensors, Actuators)
 */
public class BusMessage extends EEDiscreteEvent {

	/**
	 * Message to be transmitted.
	 */
	private Object message;

	/**
	 * Message length in bytes
	 */
	private int messageLen;


	private int transmittedBytes;

	private boolean transmitted;

	/**
	 * Id of the sender of this message
	 */
	private UUID controllerID;

	/**
	 * The type of the message
	 */
	private BusEntry messageID;

	/**
	 * Time when the this message arrives at the target
	 */
	private Instant finishTime;

	private boolean error;

	/**
	 * Ids of components this message has traversed. (Avoid infinite loops)
	 */
	private Set<UUID> seenComponentIds;

	/**
	 * Random number generator to determine a bit error
	 */
	Random bitError = new Random();

	public BusMessage(Object message, int messageLen, BusEntry messageID, Instant eventTime, UUID sourceID,
			EEComponent target) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, eventTime, target);
		this.message = message;
		this.messageLen = messageLen;
		this.messageID = messageID;
		this.controllerID = sourceID;
		this.transmittedBytes = 0;
		this.transmitted = false;
		this.finishTime = Instant.EPOCH;
		this.error = false;
		this.seenComponentIds = new HashSet<UUID>();
	}

	private BusMessage(Object message, int messageLen, BusEntry messageID, Instant eventTime, UUID sourceID,
			EEComponent target, Set<UUID> seenComponentsIds) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, eventTime, target);
		this.message = message;
		this.messageLen = messageLen;
		this.messageID = messageID;
		this.controllerID = sourceID;
		this.transmittedBytes = 0;
		this.transmitted = false;
		this.finishTime = Instant.EPOCH;
		this.error = false;
		this.seenComponentIds = seenComponentsIds;
	}

	public BusMessage(BusMessage busMessage) {
		super(EEDiscreteEventTypeEnum.BUSMESSAGE, busMessage.getEventTime(), busMessage.getTarget());
		this.message = busMessage.message;
		this.messageLen = busMessage.messageLen;
		this.transmittedBytes = busMessage.transmittedBytes;
		this.transmitted = busMessage.transmitted;
		this.controllerID = busMessage.controllerID;
		this.messageID = busMessage.messageID;
		this.finishTime = busMessage.finishTime;
		this.error = busMessage.error;
		this.bitError = busMessage.bitError;
		this.seenComponentIds = busMessage.seenComponentIds;
	}

	public Instant getFinishTime() {
		return finishTime;
	}

	public void setFinishTime(Instant finishTime) {
		this.finishTime = finishTime;
	}

	public Object getMessage() {
		return message;
	}

	public int getMessageLen() {
		return messageLen;
	}

	public UUID getControllerID() {
		return controllerID;
	}

	public BusEntry getMessageID() {
		return messageID;
	}

	public int getTransmittedBytes() {
		return transmittedBytes;
	}

	public boolean isTransmitted() {
		return transmitted;
	}

	public boolean isError() {
		return error;
	}

	public int getRemainingBytes() {
		return this.messageLen - this.transmittedBytes;
	}

	/**
	 * Transmit maximum number of bytes bytes of this message.
	 * @param bytes Maximum number of bytes to be transmitted
	 * @param bitErrorRate Rate of bit errors
	 * @return Actual transmitted bytes
	 */
	public int transmitBytes(int bytes, double bitErrorRate) {
		int res = -1;
		if (bytes >= 0) {
			if (this.messageLen - this.transmittedBytes - bytes <= 0) {
				transmitted = true;
				res = messageLen - this.transmittedBytes;
				transmittedBytes = messageLen;
			} else {
				transmittedBytes += bytes;
				res = bytes;
			}
			// bound defines the probability of a bit error (1/bound)
			int bound = Math.max(1, (int) ((1 / bitErrorRate) / (8 * res)));
			if (this.bitError.nextInt(bound) == 0) {
				this.error = true;
			}
		}
		return res;
	}

	/**
	 * Creates a new message with target as destination and the finish time of this event as event time.
	 * @param target Target of the new message
	 * @return New BusMessage with the updated target and event time.
	 */
	public BusMessage forwardTo(EEComponent target) {
		if(!this.transmitted) {
			throw new IllegalArgumentException("Only transmitted messages can be forwarded.");
		}
		else if(target == null){
			throw new NullArgumentException();
		}
		else {
			return new BusMessage(this.message, this.messageLen, this.messageID, this.finishTime, this.getTarget().getId(),
					target, this.seenComponentIds);
		}
	}
	
	public boolean hasTraversed(Bus bus) {
		return !this.seenComponentIds.add(bus.getId());
	}
	
	@Override
	public String toString() {
		return super.toString() 
				+ "; Message: " + message
				+ "; Message length: " + messageLen
				+ "; Message id: " + messageID
				+ "; Sender Id: " + controllerID
				+ "; Transmitted Bytes: " + transmittedBytes
				+ "; Transmitted: " + transmitted
				+ "; Finisht Time: " + finishTime
				+ "; Error: " + error
				+ "; Seen Components: " + seenComponentIds;
	}

}

/**
 * Used for sorting in descending order of messageId ordinal value
 */
class BusMessageComparatorIdDesc implements Comparator<BusMessage> {
	public int compare(BusMessage a, BusMessage b) {
		return b.getMessageID().compareTo(a.getMessageID());
	}
}
