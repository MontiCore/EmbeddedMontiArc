/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.jfree.util.Log;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;

import java.time.Duration;
import java.time.Instant;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.UUID;

public class FlexRay extends Bus {

	/**
	 * Size of frame header in byte
	 */
	protected static final int HEADER_SIZE = 5;

	/**
	 * Size of frame trailer in byte
	 */
	protected static final int TRAILER_SIZE = 3;

	/**
	 * Maximum bytes of payload that fit in one slot
	 */
	protected static final int MAX_SLOT_PAYLOAD = 16;

	/**
	 * Maximum bytes that fit in one slot
	 */
	protected static final int MAX_SLOT_SIZE = FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD + FlexRay.TRAILER_SIZE;

	/**
	 * Number of static slots per station
	 */
	private static final int STATIC_SLOTS = 1;

	/**
	 * Total bytes of payload of a controller in a static segment
	 */
	protected static final int CONTROLLER_STATIC_PAYLOAD = FlexRay.STATIC_SLOTS * FlexRay.MAX_SLOT_PAYLOAD;

	/**
	 * Number of dynamic slots per cycle
	 */
	protected static final int DYNAMIC_SLOTS = 4;

	/**
	 * Total bytes dynamic payload per cycle
	 */
	protected static final int TOTAL_DYNAMIC_PAYLOAD = FlexRay.MAX_SLOT_PAYLOAD * FlexRay.DYNAMIC_SLOTS;

	/**
	 * Total size of the dynamic segment in bytes
	 */
	protected static final int DYNAMIC_SEGMENT_SIZE = FlexRay.DYNAMIC_SLOTS
			* (FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD + FlexRay.TRAILER_SIZE);

	/**
	 * Comparator that sorts {@link BusMessage} ascending according to their id
	 */
	private static final BusMessageComparatorIdDesc COMP_ID_DESC = new BusMessageComparatorIdDesc();

	/**
	 * The mode in which this bus is configured
	 */
	private FlexRayOperationMode mode = FlexRayOperationMode.REDUNDANCY;

	private Map<UUID, PriorityQueue<BusMessage>> messagesByControllerId = new LinkedHashMap<UUID, PriorityQueue<BusMessage>>();

	/**
	 * Point in time where the partialCycle has begun
	 */
	private Instant lastPartialCycleStart = Instant.EPOCH;

	/**
	 * Amount of bytes that were send in the static segment during the last partial
	 * cycle (including overhead)
	 */
	private int lastPartialStaticSegmentBytes = 0;

	/**
	 * Amount of bytes that were send in the dynamic segment during the last partial
	 * cycle (including overhead)
	 */
	private int lastPartialDynamicSegmentBytes = 0;

	private Instant currentTime = Instant.EPOCH;

	public FlexRay(EESimulator simulator) {
		super(simulator);
	}

	@Override
	public void registerComponent(EEComponent component) {
		if (component.getId() == null) {
			throw new IllegalArgumentException("ID of Component " + component.toString()
					+ " is null. Only fully initialized components can be registered at " + this.toString() + ".");
		}
		if (messagesByControllerId.containsKey(component.getId())) {
			throw new IllegalArgumentException(
					"Component " + component.toString() + " is already registered at " + this.toString() + ".");
		} else {
			super.registerComponent(component);
			messagesByControllerId.put(component.getId(), new PriorityQueue<BusMessage>(COMP_ID_DESC));
		}

	}

	/**
	 * @return total amount of bytes in a static segment
	 */
	private int getTotalStaticSegmentSize() {
		return (FlexRay.HEADER_SIZE + FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TRAILER_SIZE)
				* (this.getConnectedComponents().size() * FlexRay.STATIC_SLOTS);
	}

	@Override
	public OperationMode getOperationMode() {
		return mode;
	}

	public void setOperationMode(FlexRayOperationMode mode) {
		this.mode = mode;
	}

	/**
	 * @return The duration of a slot
	 */
	Duration getSlotSize() {
		long mikroBits = ((long) (HEADER_SIZE + TRAILER_SIZE + MAX_SLOT_PAYLOAD)) * 8l * 1000000l;
		double nanoseconds = mikroBits / ((double) mode.getDataRate());
		return Duration.ofNanos((long) Math.ceil(nanoseconds));
	}

	/**
	 * @return The duration of the static segment
	 */
	Duration getStaticSegmentSize() {
		return this.getSlotSize().multipliedBy(this.messagesByControllerId.size() * FlexRay.STATIC_SLOTS);
	}

	/**
	 * @return The duration of the dynamic segment
	 */
	Duration getDynamicSegmentSize() {
		long nanoseconds = this.getSlotSize().toNanos() * FlexRay.DYNAMIC_SLOTS;
		return Duration.ofNanos(nanoseconds);
	}

	/**
	 * @return The duration of a cycle
	 */
	public Duration getCycleTime() {
		return this.getSlotSize()
				.multipliedBy((this.messagesByControllerId.size() * FlexRay.STATIC_SLOTS) + FlexRay.DYNAMIC_SLOTS);
	}

	protected Map<UUID, PriorityQueue<BusMessage>> getMessagesByControllerId() {
		return this.messagesByControllerId;
	}

	@Override
	public Instant getCurrentTime() {
		return this.currentTime;
	}

	@Override
	public BusType getBusType() {
		return BusType.FLEXRAY;
	}

	/**
	 * Determines if messagesByControllerId contains no messages
	 *
	 * @return true if messagesByControllerId contains at least one message
	 */
	@Override
	protected boolean hasMessages() {
		return getNextDynamicMessage() != null;
	}

	@Override
	protected void simulateUntil(Instant endTime) {
		// TODO do not waste time when messages are empty
		Duration duration = Duration.between(this.currentTime, endTime);
		//only simulate full bytes
		long byteTransmissionTimeNs =calculateTransmissionTime(1);
		int toTransmit = Math.toIntExact(duration.toNanos()/byteTransmissionTimeNs);
		Log.info("Transmit bytes: " + toTransmit);
		toTransmit = this.finishLastCycle(toTransmit);

		Log.info("Transmit bytes after finishing last incomplete cycle: " + toTransmit);

		int totalBytes = toTransmit;

		if (toTransmit > 0) {
			Instant segmentStartTime = this.currentTime;
			Duration staticSegmentDuration = this.getStaticSegmentSize();
			Duration dynamicSegmentDuration = this.getDynamicSegmentSize();

			long fullCycles = (toTransmit / (this.getTotalStaticSegmentSize() + DYNAMIC_SEGMENT_SIZE));

			// go through simulation cycle by cycle
			for (int cycle = 0; cycle < fullCycles; cycle++) {
				// sent the frames for this cycle
				this.fillStaticSegment(segmentStartTime);
				segmentStartTime = segmentStartTime.plusNanos(staticSegmentDuration.toNanos());
				this.fillDynamicSegment(segmentStartTime);
				segmentStartTime = segmentStartTime.plusNanos(dynamicSegmentDuration.toNanos());
				toTransmit -= (this.getTotalStaticSegmentSize() + DYNAMIC_SEGMENT_SIZE);
			}

			this.fillIncompleteStaticSegment(segmentStartTime, toTransmit, 0);
			if (toTransmit <= this.getTotalStaticSegmentSize()) {
				this.lastPartialStaticSegmentBytes = toTransmit;
				this.lastPartialDynamicSegmentBytes = 0;
				this.lastPartialCycleStart = this.currentTime.plus(this.getCycleTime().multipliedBy(fullCycles));
			} else {
				toTransmit -= this.getTotalStaticSegmentSize();
				segmentStartTime = segmentStartTime.plusNanos(staticSegmentDuration.toNanos());
				this.fillIncompleteDynamicSegment(segmentStartTime, toTransmit, 0);
				this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
				this.lastPartialDynamicSegmentBytes = toTransmit;
				this.lastPartialCycleStart = this.currentTime.plus(this.getCycleTime().multipliedBy(fullCycles));
			}
		}
		this.currentTime = this.currentTime.plusNanos(calculateTransmissionTime(totalBytes));

	}

	@Override
	protected void registerMessage(BusMessage msg) {
		if (!targetsByMessageId.containsKey(msg.getMessageID())) {
			Log.debug("Message has no target in Bus " + this.toString());
		}
		if (!messagesByControllerId.containsKey(msg.getControllerID())) {
			throw new IllegalArgumentException("Message send by unknown controller @" + this.toString());
		} else if (!msg.getTarget().getId().equals(this.getId())) {
			throw new IllegalArgumentException("Message with incorrect target @" + this.toString());
		} else if(msg.getMessageLen() <= 0) {
			throw new IllegalArgumentException("Message length can not be 0 or negative. Message length was + " + msg.getMessageLen());
		}else{
			PriorityQueue<BusMessage> controllerMsgs = messagesByControllerId.get(msg.getControllerID());
			controllerMsgs.add(msg);
			messagesByControllerId.put(msg.getControllerID(), controllerMsgs);
		}
	}

	@Override
	protected Instant getNextFinishTime() {
		//get first messages of each controller
		Map<UUID, BusMessage> firstMsgByControllerId = new LinkedHashMap<UUID, BusMessage>();
		for (Map.Entry<UUID, PriorityQueue<BusMessage>> controllerMsgsById : this.messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = controllerMsgsById.getValue();
			BusMessage firstMsg = controllerMessages.peek();
			while(firstMsg != null && firstMsg.isTransmitted()){
				controllerMessages.poll();
				firstMsg = controllerMessages.peek();
			}
			if(firstMsg == null) {
				firstMsgByControllerId.put(controllerMsgsById.getKey(), null);
			}else {
				firstMsgByControllerId.put(controllerMsgsById.getKey(), new BusMessage(firstMsg));
			}
		}

		//determine highest priority message
		BusMessage highestPrio = null;
		for (BusMessage msg : firstMsgByControllerId.values()) {
			if (msg != null && (highestPrio == null || COMP_ID_DESC.compare(highestPrio, msg) > 0)) {
				highestPrio = msg;
			}
		}

		//calculate earliest finish time
		Instant earliestFinishTime = Instant.MAX;
		Instant time = Instant.from(currentTime);
		// finish last incomplete cycle
		if (lastPartialStaticSegmentBytes < this.getTotalStaticSegmentSize() && lastPartialStaticSegmentBytes > 0) {
			Log.info("Start from incomplete static segment with " + lastPartialStaticSegmentBytes + " bytes");
			Pair<Integer, Instant> pair = mockFillIncompleteStaticSegement(firstMsgByControllerId);
			if (pair.getRight().isBefore(earliestFinishTime)) {
				earliestFinishTime = pair.getRight();
				Log.info("New earliest finished msg in incomplete static segment with: " + pair.getLeft()
						+ " transmittedBytes");
			} else {
				int transmittedBytes = pair.getLeft();
				int fullSlots = highestPrio.getRemainingBytes() / FlexRay.MAX_SLOT_PAYLOAD;
				int remainingBytes = highestPrio.getRemainingBytes() % FlexRay.MAX_SLOT_PAYLOAD;
				if (fullSlots < FlexRay.DYNAMIC_SLOTS) {
					transmittedBytes += (fullSlots * FlexRay.MAX_SLOT_SIZE);
					if (remainingBytes > 0) {
						transmittedBytes += FlexRay.HEADER_SIZE + remainingBytes;
					}
					earliestFinishTime = time.plusNanos(calculateTransmissionTime(transmittedBytes));
					Log.info("New earliest finished msg + " + highestPrio
							+ " finished in dynamic segment of incomplete cycle with: " + pair.getLeft()
							+ " transmittedBytes");
				} else {
					highestPrio.transmitBytes(FlexRay.TOTAL_DYNAMIC_PAYLOAD, 0);
					transmittedBytes += FlexRay.DYNAMIC_SEGMENT_SIZE;
					Log.info("Transmitted bytes in incomplete segment: " + transmittedBytes);
					time = time.plusNanos(calculateTransmissionTime(transmittedBytes));
				}
			}
		} else if (lastPartialStaticSegmentBytes > 0 && lastPartialDynamicSegmentBytes < FlexRay.DYNAMIC_SEGMENT_SIZE
				&& lastPartialDynamicSegmentBytes >= 0) {
			Log.info("Start from incomplete dynamic segment with " + lastPartialDynamicSegmentBytes + " bytes");
			Pair<Integer, Instant> pair = mockFillIncompleteDynamicSegment(highestPrio);
			if (pair.getRight().isBefore(earliestFinishTime)) {
				earliestFinishTime = pair.getRight();
				Log.info("New earliest finished msg: " + highestPrio
						+ " finished in incomplete dynamic segment with: " + pair.getLeft() + " transmittedBytes");
			} else {
				Log.info("Transmitted bytes in incomplete segment: " + pair.getLeft());
				time = time.plusNanos(calculateTransmissionTime(pair.getLeft()));
			}
		}

		if (earliestFinishTime.equals((Instant.MAX))) {
			Duration slotOffset = Duration.ZERO;
			for (BusMessage msg : firstMsgByControllerId.values()) {
				if (msg != null) {
					int remainingBytes = msg.getRemainingBytes();
					int completeCycles = 0;
					int completeSlots = 0;
					long partialSlotNs = 0;
					if (msg.getId() == highestPrio.getId()) {
						// transmitted during static and dynamic segments
						completeCycles = remainingBytes
								/ (FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TOTAL_DYNAMIC_PAYLOAD);
						remainingBytes = remainingBytes
								% (FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TOTAL_DYNAMIC_PAYLOAD);
					} else {
						// transmitted only during static segments
						completeCycles = remainingBytes / (FlexRay.CONTROLLER_STATIC_PAYLOAD);
						remainingBytes = remainingBytes % (FlexRay.CONTROLLER_STATIC_PAYLOAD);
					}
					//last cycle might not be used fully but only until the static segment of controller
					if(remainingBytes == 0 ){
						completeCycles = Math.max(completeCycles - 1, 0);
						if(msg.getMessageID() != highestPrio.getMessageID()){
							completeSlots = FlexRay.STATIC_SLOTS;
						}
						else{
							completeSlots = FlexRay.STATIC_SLOTS + FlexRay.DYNAMIC_SLOTS;
						}
					}
					else{
						completeSlots = remainingBytes / FlexRay.MAX_SLOT_PAYLOAD;
						remainingBytes = remainingBytes % FlexRay.MAX_SLOT_PAYLOAD;
						//partial slot is used
						if(remainingBytes > 0){
							partialSlotNs = this.calculateTransmissionTime(FlexRay.HEADER_SIZE + remainingBytes);

						}
					}
					 //complete cycles and incomplete slots
					Instant finishTime = time.plus(getCycleTime().multipliedBy(completeCycles)).plusNanos(partialSlotNs);
					 //complete slots
					 if (completeSlots < FlexRay.STATIC_SLOTS || (completeSlots == FlexRay.STATIC_SLOTS && remainingBytes == 0)) {
						finishTime = finishTime.plus(slotOffset).plus(getSlotSize().multipliedBy(completeSlots));
						Log.info("Message: " + msg + "finished in static slot @" + finishTime);
					} 
					// only possible for message with highest priority
					else {
						finishTime = finishTime.plus(getStaticSegmentSize())
								.plus(getSlotSize().multipliedBy(completeSlots - 1));
						Log.info("Message: " + msg + "finished in dynamic slot@" + finishTime);
					}
					if (finishTime.isBefore(earliestFinishTime)) {
						earliestFinishTime = finishTime;
						Log.info("New earliest finished msg: " + msg + " finished after: " + completeCycles
								+ " cycles; " + completeSlots + " slots; " + remainingBytes + " remaining bytes");
					}
				}
				slotOffset = slotOffset.plus(getSlotSize());
			}
		}
		if (earliestFinishTime.isBefore(Instant.MAX)) {
			return earliestFinishTime;
		} else {
			return currentTime;
		}
	}

	/**
	 * Finish of the last cycle that was not completely simulated. Do nothing
	 * otherwise.
	 * 
	 * @param toTransmit amount of bytes for which the bus is simulated
	 * @return duration minus the amount of time it took the finish of the last
	 *         cycle.
	 */
	private int finishLastCycle(int toTransmit) {
		int totalBytes = toTransmit;
		if (lastPartialStaticSegmentBytes < this.getTotalStaticSegmentSize() && lastPartialStaticSegmentBytes > 0) {
			Log.info("Start simulation from incomplete static slot with " + lastPartialStaticSegmentBytes + "bytes transmitted"
					+ "; transmit " + toTransmit + " now");
			int remainingBytes = this.getTotalStaticSegmentSize() - this.lastPartialStaticSegmentBytes;
			if (toTransmit < remainingBytes) {
				this.fillIncompleteStaticSegment(this.lastPartialCycleStart, toTransmit, lastPartialStaticSegmentBytes);
				this.currentTime = this.currentTime.plusNanos(calculateTransmissionTime(totalBytes));
				this.lastPartialDynamicSegmentBytes = 0;
				this.lastPartialStaticSegmentBytes += toTransmit;
				toTransmit = 0;
			} else {
				this.fillIncompleteStaticSegment(this.lastPartialCycleStart, remainingBytes, lastPartialStaticSegmentBytes);
				toTransmit -= remainingBytes;
				remainingBytes = FlexRay.DYNAMIC_SEGMENT_SIZE;
				if (toTransmit < remainingBytes) {
					this.fillIncompleteDynamicSegment(this.lastPartialCycleStart.plus(this.getStaticSegmentSize()), toTransmit, 0);
					this.currentTime = this.currentTime.plusNanos(calculateTransmissionTime(totalBytes));
					this.lastPartialDynamicSegmentBytes = toTransmit;
					this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
					toTransmit = 0;
				} else {
					this.fillDynamicSegment(this.lastPartialCycleStart.plus(this.getStaticSegmentSize()));
					this.currentTime = this.lastPartialCycleStart.plus(this.getCycleTime());
					this.lastPartialCycleStart = Instant.EPOCH;
					this.lastPartialDynamicSegmentBytes = 0;
					this.lastPartialStaticSegmentBytes = 0;
					toTransmit -= remainingBytes;
				}
			}
		} else if (lastPartialStaticSegmentBytes > 0 && lastPartialDynamicSegmentBytes < FlexRay.DYNAMIC_SEGMENT_SIZE
				&& lastPartialDynamicSegmentBytes >= 0) {
			Log.info("Start simulation from incomplete dynamic slot with " + lastPartialDynamicSegmentBytes + "bytes transmitted");
			int remainingBytes = FlexRay.DYNAMIC_SEGMENT_SIZE - this.lastPartialDynamicSegmentBytes;
			if (toTransmit < remainingBytes) {
				this.fillIncompleteDynamicSegment(this.lastPartialCycleStart.plus(this.getStaticSegmentSize()), toTransmit, lastPartialDynamicSegmentBytes);
				this.currentTime = this.currentTime.plusNanos(calculateTransmissionTime(totalBytes));
				this.lastPartialDynamicSegmentBytes += toTransmit;
				this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
				toTransmit = 0;
			} else {
				this.fillIncompleteDynamicSegment(this.lastPartialCycleStart.plus(this.getStaticSegmentSize()), remainingBytes, lastPartialDynamicSegmentBytes);
				this.currentTime = this.lastPartialCycleStart.plus(this.getCycleTime());
				this.lastPartialCycleStart = Instant.EPOCH;
				this.lastPartialDynamicSegmentBytes = 0;
				this.lastPartialStaticSegmentBytes = 0;
				toTransmit -= remainingBytes;
			}
		}
		return toTransmit;
	}

	/**
	 * Fill a complete static segment
	 * 
	 * @param segmentStartTime the time at which the segment was started.
	 */
	private void fillStaticSegment(Instant segmentStartTime) {
		int transmittedBytes = 0;
		for (Map.Entry<UUID, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			transmittedBytes += FlexRay.HEADER_SIZE;
			PriorityQueue<BusMessage> msgs = fillStaticSlot(segmentStartTime, entry.getValue(), transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD, 0);
			entry.setValue(msgs);
			transmittedBytes += FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TRAILER_SIZE;
		}
	}

	/**
	 * Fill an incomplete (it was already started or is not finished) static
	 * segment.
	 * 
	 * @param segmentStartTime the time at which the segment was started
	 * @param bytesToSend      the amount of bytes to send (including overhead)
	 * @param startByte        the byte from which the segment is filled (including
	 *                         overhead)
	 */
	private void fillIncompleteStaticSegment(Instant segmentStartTime, int bytesToSend, int startByte) {
		int totalTransmittedBytes = startByte;
		for (Map.Entry<UUID, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			if (bytesToSend > 0) {
				for (int i = 0; i < FlexRay.STATIC_SLOTS; i++) {
					if (startByte > FlexRay.MAX_SLOT_SIZE) {
						startByte -= FlexRay.MAX_SLOT_SIZE;
					} else if (startByte > (FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD)) {
						startByte -= FlexRay.HEADER_SIZE;
						startByte -= FlexRay.MAX_SLOT_PAYLOAD;
						bytesToSend -= (FlexRay.TRAILER_SIZE - startByte);
						totalTransmittedBytes += (FlexRay.TRAILER_SIZE - startByte);
						startByte = 0;
					} else if (startByte > FlexRay.HEADER_SIZE) {
						startByte -= FlexRay.HEADER_SIZE;
						int controllerBytes = Math.min(bytesToSend, FlexRay.MAX_SLOT_PAYLOAD - startByte);
						PriorityQueue<BusMessage> msgs = this.fillStaticSlot(segmentStartTime, entry.getValue(),
								totalTransmittedBytes, controllerBytes, startByte);
						entry.setValue(msgs);
						startByte = 0;
						bytesToSend -= controllerBytes;
						totalTransmittedBytes += controllerBytes;
						bytesToSend -= FlexRay.TRAILER_SIZE;
						totalTransmittedBytes += FlexRay.TRAILER_SIZE;
					} else if (startByte > 0) {
						bytesToSend -= FlexRay.HEADER_SIZE - startByte;
						totalTransmittedBytes += FlexRay.HEADER_SIZE - startByte;
						startByte = 0;
						int controllerBytes = Math.min(bytesToSend, FlexRay.MAX_SLOT_PAYLOAD);
						PriorityQueue<BusMessage> msgs = this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes,
								0);
						entry.setValue(msgs);
						bytesToSend -= controllerBytes;
						totalTransmittedBytes += controllerBytes;
						bytesToSend -= FlexRay.TRAILER_SIZE;
						totalTransmittedBytes += FlexRay.TRAILER_SIZE;
					} else {
						bytesToSend -= FlexRay.HEADER_SIZE;
						totalTransmittedBytes += FlexRay.HEADER_SIZE;
						startByte = 0;
						int controllerBytes = Math.min(bytesToSend, FlexRay.MAX_SLOT_PAYLOAD);
						PriorityQueue<BusMessage> msgs = this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes,
								0);
						entry.setValue(msgs);
						bytesToSend -= controllerBytes;
						totalTransmittedBytes += controllerBytes;
						bytesToSend -= FlexRay.TRAILER_SIZE;
						totalTransmittedBytes += FlexRay.TRAILER_SIZE;
					}
				}

			}
		}
	}

	/**
	 * Fill a static slot for one controller.
	 * 
	 * @param segmentStartTime the time at which the segment has started
	 * @param msgs             the messages that belong to the controller
	 * @param transmittedBytes the bytes that were already transmitted during this
	 *                         segment (including overhead)
	 * @param slotPayload      the payload that should be transmitted during the
	 *                         slot
	 * @param startByte        the byte from which the slot is filled
	 * @return the unfinished messages of the controller
	 */
	private PriorityQueue<BusMessage> fillStaticSlot(Instant segmentStartTime, PriorityQueue<BusMessage> msgs,
			int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = ((startByte + slotPayload) % FlexRay.MAX_SLOT_PAYLOAD) == 0;
		while (!msgs.isEmpty() && slotPayload > 0) {
			BusMessage message = msgs.peek();
			if (!message.isTransmitted()) {
				int messageBytes = message.transmitBytes(slotPayload, mode.getBitErrorRate());
				Log.info("Transmitted " + messageBytes + " bytes now in static slot from " + message);
				slotPayload -= messageBytes;
				transmittedBytes += messageBytes;
				if (messageBytes >= 0) {
					if (message.isTransmitted()) {
						// remove the message
						msgs.poll();
						if (endOfSlot && slotPayload == 0) {
							transmittedBytes += FlexRay.TRAILER_SIZE;
						}
						message.setFinishTime(
								segmentStartTime.plusNanos(this.calculateTransmissionTime(transmittedBytes)));
						Log.info(message + " transmitted with " + transmittedBytes + " (" + (transmittedBytes -lastPartialStaticSegmentBytes)+")" + " transmitted bytes in static segment");
						this.registerMessageAtSimulator(message);
					}
				} else {
					Log.warn("Error transmitting message in static segment of bus: " + this.getId());
				}
			} else {
				msgs.poll();
			}
		}
		return msgs;
	}

	private ImmutablePair<Integer, Instant> mockFillIncompleteStaticSegement(
			Map<UUID, BusMessage> firstMessageByControllerId) {
		Instant finishTime = Instant.MAX;
		int startByte = lastPartialStaticSegmentBytes;
		int transmittedBytes = 0;
		for (BusMessage firstMessage : firstMessageByControllerId.values()) {
			for (int i = 0; i < FlexRay.STATIC_SLOTS; i++) {
				if (startByte > FlexRay.MAX_SLOT_SIZE) {
					startByte -= FlexRay.MAX_SLOT_SIZE;
				} else if (startByte > (FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD)) {
					startByte -= FlexRay.HEADER_SIZE;
					startByte -= FlexRay.MAX_SLOT_PAYLOAD;
					transmittedBytes += (FlexRay.TRAILER_SIZE - startByte);
					startByte = 0;
				} else if (startByte > FlexRay.HEADER_SIZE) {
					startByte -= FlexRay.HEADER_SIZE;
					finishTime = mockFillSlot(firstMessage, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD - startByte);
					transmittedBytes += FlexRay.MAX_SLOT_PAYLOAD - startByte + FlexRay.TRAILER_SIZE;
					startByte = 0;
				} else if (startByte > 0) {
					transmittedBytes += FlexRay.HEADER_SIZE - startByte;
					startByte = 0;
					finishTime = mockFillSlot(firstMessage, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD - startByte);
					transmittedBytes += FlexRay.MAX_SLOT_PAYLOAD - startByte + FlexRay.TRAILER_SIZE;
					startByte = 0;
				} else {
					transmittedBytes += FlexRay.HEADER_SIZE;
					finishTime = mockFillSlot(firstMessage, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD - startByte);
					transmittedBytes += FlexRay.MAX_SLOT_PAYLOAD - startByte + FlexRay.TRAILER_SIZE;
				}
			}
			if (finishTime.isBefore(Instant.MAX)) {
				return new ImmutablePair<Integer, Instant>(transmittedBytes, finishTime);
			}
		}
		return new ImmutablePair<Integer, Instant>(transmittedBytes, finishTime);
	}

	private Instant mockFillSlot(BusMessage msg, int transmittedBytes, int slotPayload) {
		Instant finishTime = Instant.MAX;
		if (msg != null) {
			int msgBytes = msg.transmitBytes(slotPayload, 0);
			transmittedBytes += msgBytes;
			if (msg.isTransmitted()) {
				Log.info("Message: " + msg + "transmitted after " + transmittedBytes + "bytes");
				if(msgBytes == slotPayload) {
					finishTime = currentTime.plusNanos(calculateTransmissionTime(transmittedBytes + FlexRay.TRAILER_SIZE));
				}else {
					finishTime = currentTime.plusNanos(calculateTransmissionTime(transmittedBytes));
				}	
			} 
		}
		return finishTime;
	}

	/**
	 * Fill a complete dynamic segment
	 * 
	 * @param segmentStartTime the time at which the segment was started.
	 */
	private void fillDynamicSegment(Instant segmentStartTime) {
		int bytesToSend = FlexRay.DYNAMIC_SEGMENT_SIZE;
		int transmittedBytes = 0;
		while (bytesToSend > 0) {
			bytesToSend -= FlexRay.HEADER_SIZE;
			transmittedBytes += FlexRay.HEADER_SIZE;
			int slotPayload = FlexRay.MAX_SLOT_PAYLOAD;
			this.fillDynamicSlot(segmentStartTime, transmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			transmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}
	}

	/**
	 * Fill an incomplete (it was already started or is not finished) dynamic
	 * segment.
	 * 
	 * @param segmentStartTime the time at which the segment was started
	 * @param bytesToSend      the amount of bytes to send (including overhead)
	 * @param startByte        the byte from which the segment is filled (including
	 *                         overhead)
	 */
	private void fillIncompleteDynamicSegment(Instant segmentStartTime, int bytesToSend, int startByte) {
		int totalTransmittedBytes = startByte;
		int fullSlots = startByte / FlexRay.MAX_SLOT_SIZE;
		startByte -= (fullSlots * FlexRay.MAX_SLOT_SIZE);
		if (startByte > FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD) {
			startByte -= FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD;
			bytesToSend -= FlexRay.TRAILER_SIZE - startByte;
			totalTransmittedBytes += FlexRay.TRAILER_SIZE - startByte;
		} else if (startByte > FlexRay.HEADER_SIZE) {
			startByte -= FlexRay.HEADER_SIZE;
			int slotPayload = Math.min(FlexRay.MAX_SLOT_PAYLOAD - startByte, bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, startByte);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		} else if (startByte > 0) {
			bytesToSend -= FlexRay.HEADER_SIZE - startByte;
			totalTransmittedBytes += FlexRay.HEADER_SIZE - startByte;
			int slotPayload = Math.min(FlexRay.MAX_SLOT_PAYLOAD, bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}

		while (bytesToSend > 0) {
			bytesToSend -= FlexRay.HEADER_SIZE;
			totalTransmittedBytes += FlexRay.HEADER_SIZE;
			int slotPayload = Math.min(FlexRay.MAX_SLOT_PAYLOAD, bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}
	}

	/**
	 * Fill a dynamic slot for one controller.
	 * 
	 * @param segmentStartTime the time at which the segment has started
	 * @param transmittedBytes the bytes that were already transmitted during this
	 *                         segment (including overhead)
	 * @param slotPayload      the payload that should be transmitted during the
	 *                         slot
	 * @param startByte        the byte from which the slot is filled
	 */
	private void fillDynamicSlot(Instant segmentStartTime, int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = ((startByte + slotPayload) % FlexRay.MAX_SLOT_PAYLOAD) == 0;
		BusMessage cur = this.getNextDynamicMessage();
		while (cur != null && slotPayload > 0) {
			int messageBytes = cur.transmitBytes(slotPayload, mode.getBitErrorRate());
			slotPayload -= messageBytes;
			transmittedBytes += messageBytes;
			Log.info("Transmitted " + messageBytes + " bytes now in dynamic slot from " + cur);
			if (messageBytes >= 0) {
				if (cur.isTransmitted()) {
					if (endOfSlot && slotPayload == 0) {
						transmittedBytes += FlexRay.TRAILER_SIZE;
					}
					long nanoseconds = this.calculateTransmissionTime(transmittedBytes);
					cur.setFinishTime(segmentStartTime.plusNanos(nanoseconds));
					Log.info(cur + " transmitted with " + transmittedBytes + " (" + (transmittedBytes -lastPartialDynamicSegmentBytes)+")" + " transmitted bytes in dynamic segment");
					this.removeFirstMessage(cur);
					this.registerMessageAtSimulator(cur);
					cur = this.getNextDynamicMessage();
				}
			} else {
				Log.warn("Error transmitting message in dynamic segment of bus: " + this.getId());
			}
		}
	}

	private Pair<Integer, Instant> mockFillIncompleteDynamicSegment(BusMessage msg) {
		Instant finishTime = Instant.MAX;
		int startByte = lastPartialDynamicSegmentBytes;
		int transmittedBytes = 0;
		int fullSlots = startByte / FlexRay.MAX_SLOT_SIZE;
		startByte -= (fullSlots * FlexRay.MAX_SLOT_SIZE);
		int remainingSlots = FlexRay.DYNAMIC_SLOTS - fullSlots;
		if(startByte != 0){
			remainingSlots -= 1;
		}

		// finish off incomplete slot
		if (startByte > FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD) {
			startByte -= FlexRay.HEADER_SIZE + FlexRay.MAX_SLOT_PAYLOAD;
			transmittedBytes += FlexRay.TRAILER_SIZE - startByte;
			startByte = 0;
		} else if (startByte > FlexRay.HEADER_SIZE) {
			startByte -= FlexRay.HEADER_SIZE;
			finishTime = mockFillSlot(msg, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD - startByte);
			transmittedBytes += FlexRay.MAX_SLOT_PAYLOAD - startByte + FlexRay.TRAILER_SIZE;
			startByte = 0;
		} else if (startByte > 0) {
			transmittedBytes += FlexRay.HEADER_SIZE - startByte;
			finishTime = mockFillSlot(msg, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD);
			transmittedBytes += MAX_SLOT_PAYLOAD + FlexRay.TRAILER_SIZE;
			startByte = 0;
		}
		while (remainingSlots > 0 && !finishTime.isBefore(Instant.MAX)) {
			remainingSlots--;
			transmittedBytes += FlexRay.HEADER_SIZE;
			finishTime = mockFillSlot(msg, transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD);
			transmittedBytes += FlexRay.MAX_SLOT_PAYLOAD + FlexRay.TRAILER_SIZE;
		}
		return new ImmutablePair<Integer, Instant>(transmittedBytes, finishTime);
	}

	/**
	 * @return the message with the highest overall priority
	 */
	private BusMessage getNextDynamicMessage() {
		BusMessage res = null;
		for (Map.Entry<UUID, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			BusMessage msg = null;
			while (!controllerMessages.isEmpty() && (msg == null || msg.isTransmitted())) {
				// get next
				msg = controllerMessages.peek();
				if (!msg.isTransmitted()) {
					if (res == null || COMP_ID_DESC.compare(res, msg) > 0) {
						res = msg;
					}
				} else {
					// remove transmitted
					controllerMessages.poll();
				}
			}
			entry.setValue(controllerMessages);
		}
		return res;
	}
	
	private void removeFirstMessage(BusMessage msg) {
		for (Map.Entry<UUID, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			if (!controllerMessages.isEmpty() && (msg.getId().equals(controllerMessages.peek().getId()))) {
				controllerMessages.poll();
			}
			entry.setValue(controllerMessages);
		}
	}

}
