
/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
import java.time.Duration;
import java.time.temporal.ChronoUnit;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

import org.jfree.util.Log;

import commons.simulation.DiscreteEvent;

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
	protected static final int MAX_SLOT_PAYLOAD = 254;

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
	 * Comparator that sorts {@link BusMessages} ascending according to their id
	 */
	private static final BusMessageComparatorIdDesc COMP_ID_DESC = new BusMessageComparatorIdDesc();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

	private Map<String, PriorityQueue<BusMessage>> messagesByControllerId = new LinkedHashMap<String, PriorityQueue<BusMessage>>();
	
	/**
	 * Duration of the last partial cycle that was simulated
	 */
	private Duration lastPartialCycleDuration = Duration.ZERO;

	/**
	 * Amount of bytes that were send in the static segment during the last partial cycle (including overhead)
	 */
	private int lastPartialStaticSegmentBytes = 0;
	
	/**
	 * Amount of bytes that were send in the dynamic segment during the last partial cycle (including overhead)
	 */
	private int lastPartialDynamicSegmentBytes = 0;

	public FlexRay(EESimulator simulator, List<EEComponent> controllers) {
		super(simulator, controllers);
		for (EEComponent controller : controllers) {
			messagesByControllerId.put(controller.getID().toString(), new PriorityQueue<BusMessage>(COMP_ID_DESC));
		}
	}

	/**
	 * @return total amount of bytes in a static segment
	 */
	private int getTotalStaticSegmentSize() {
		return (FlexRay.HEADER_SIZE + FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TRAILER_SIZE)
				* (this.connectedComponents.size() + FlexRay.STATIC_SLOTS);
	}

	public FlexRayOperationMode getMode() {
		return mode;
	}

	public void setMode(FlexRayOperationMode mode) {
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
	Duration getCycleTime() {
		return this.getSlotSize()
				.multipliedBy((this.messagesByControllerId.size() * FlexRay.STATIC_SLOTS) + FlexRay.DYNAMIC_SLOTS);
	}

	protected Map<String, PriorityQueue<BusMessage>> getMessagesByControllerId() {
		return this.messagesByControllerId;
	}

	@Override
	protected void simulateFor(Duration duration) {

		duration = this.finishLastCycle(duration);

		if (!duration.isZero()) {
			Instant segmentStartTime = this.currentTime;
			Duration staticSegmentDuration = this.getStaticSegmentSize();
			Duration dynamicSegmentDuration = this.getDynamicSegmentSize();

			long fullCycles = (duration.toNanos() / this.getCycleTime().toNanos());
			Duration partialCycleDuration = duration.minus(this.getCycleTime().multipliedBy(fullCycles));
			this.lastPartialCycleDuration = Duration.ofNanos(partialCycleDuration.toNanos());

			// go through simulation cycle by cycle
			for (int cycle = 0; cycle < fullCycles && this.messagesRegistered(); cycle++) {
				// sent the frames for this cycle
				this.fillStaticSegment(segmentStartTime);
				segmentStartTime = segmentStartTime.plusNanos(staticSegmentDuration.toNanos());
				this.fillDynamicSegment(segmentStartTime);
				segmentStartTime = segmentStartTime.plusNanos(dynamicSegmentDuration.toNanos());
			}

			int remainingBytes = Math.toIntExact((partialCycleDuration.toMillis() * this.mode.getDataRate()) / 8);
			this.fillIncompleteStaticSegment(segmentStartTime, remainingBytes, 0);
			if (remainingBytes <= this.getTotalStaticSegmentSize()) {
				this.lastPartialCycleDuration = partialCycleDuration;
				this.lastPartialStaticSegmentBytes = remainingBytes;
				this.lastPartialDynamicSegmentBytes = 0;
			} else {
				remainingBytes -= this.getTotalStaticSegmentSize();
				segmentStartTime = segmentStartTime.plusNanos(staticSegmentDuration.toNanos());
				this.fillIncompleteDynamicSegment(segmentStartTime, remainingBytes, 0);
				this.lastPartialCycleDuration = partialCycleDuration;
				this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
				this.lastPartialDynamicSegmentBytes = remainingBytes;
			}
			this.currentTime = this.currentTime.plus(duration);
		}

	}

	@Override
	protected void registerMessage(BusMessage msg) {
		if (msg.getPath().isEmpty()) {
			boolean suc = this.setPath(msg);
			if (!suc) {
				throw new IllegalArgumentException("Message send to unknown controller.");
			}
		}
		if (!messagesByControllerId.containsKey(msg.getControllerID())) {
			throw new IllegalArgumentException("Message send by unknown controller.");
		} else {
			PriorityQueue<BusMessage> controllerMsgs = messagesByControllerId.get(msg.getControllerID());
			controllerMsgs.add(msg);
			messagesByControllerId.put(msg.getControllerID(), controllerMsgs);
		}
	}

	@Override
	protected Instant getNextFinishTime() {
		PriorityQueue<BusMessage> firstMsgs = new PriorityQueue<BusMessage>(COMP_ID_DESC);
		for (PriorityQueue<BusMessage> controllerMsgs : this.messagesByControllerId.values()) {
			if (!controllerMsgs.isEmpty()) {
				firstMsgs.add(new BusMessage(controllerMsgs.peek()));
			}
		}

		int minCycle = 0;
		if (!firstMsgs.isEmpty()) {
			double toTransmit = firstMsgs.poll().getRemainingBytes();
			// transmitted during static and dynamic segments
			minCycle = (int) Math
					.ceil(toTransmit / (FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TOTAL_DYNAMIC_PAYLOAD));
			while (!firstMsgs.isEmpty()) {
				BusMessage cur = firstMsgs.poll();
				// transmitted during static segments
				int msgCycles = (int) Math.ceil(cur.getRemainingBytes() / ((double) FlexRay.CONTROLLER_STATIC_PAYLOAD));
				if (msgCycles < minCycle) {
					minCycle = msgCycles;
				}
			}
		}

		return this.currentTime.plusNanos(this.getCycleTime().toNanos() * minCycle);
	}
	
	/**
	 * Finish of the last cycle that was not completely simulated. Do nothing otherwise.
	 * @param duration amount of time for which the bus is simulated
	 * @return duration minus the amount of time it took the finish of the last cycle.
	 */
	private Duration finishLastCycle(Duration duration) {
		Instant startTime = this.currentTime.minusNanos(lastPartialCycleDuration.toNanos());
		Instant segmentStartTime = Instant.from(startTime);

		if (lastPartialStaticSegmentBytes < this.getTotalStaticSegmentSize() && lastPartialStaticSegmentBytes > 0) {
			int totalBytes = Math.toIntExact((duration.toMillis() * this.mode.getDataRate()) / 8);
			int remainingBytes = this.getTotalStaticSegmentSize() - this.lastPartialStaticSegmentBytes;
			if (totalBytes < remainingBytes) {
				this.fillIncompleteStaticSegment(segmentStartTime, totalBytes, lastPartialStaticSegmentBytes);
				this.lastPartialCycleDuration = this.lastPartialCycleDuration.plus(duration);
				this.currentTime = startTime.plus(this.lastPartialCycleDuration);
				this.lastPartialDynamicSegmentBytes = 0;
				this.lastPartialStaticSegmentBytes += totalBytes;
				duration = Duration.ZERO;
			} else {
				this.fillIncompleteStaticSegment(segmentStartTime, remainingBytes, lastPartialStaticSegmentBytes);
				segmentStartTime = segmentStartTime.plusNanos(this.getStaticSegmentSize().toNanos());
				totalBytes -= remainingBytes;
				remainingBytes = FlexRay.DYNAMIC_SEGMENT_SIZE;
				if (totalBytes < remainingBytes) {
					this.fillIncompleteDynamicSegment(segmentStartTime, totalBytes, 0);
					this.lastPartialCycleDuration = this.lastPartialCycleDuration.plus(duration);
					this.currentTime = startTime.plus(this.lastPartialCycleDuration);
					this.lastPartialDynamicSegmentBytes = totalBytes;
					this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
					duration = Duration.ZERO;
				} else {
					this.fillDynamicSegment(segmentStartTime);
					this.currentTime = startTime.plus(this.getCycleTime());
					duration = duration.plus(lastPartialCycleDuration).minus(this.getCycleTime());
					this.lastPartialCycleDuration = Duration.ZERO;
					this.lastPartialDynamicSegmentBytes = FlexRay.DYNAMIC_SEGMENT_SIZE;
					this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
				}
			}
		} else if (lastPartialStaticSegmentBytes > 0 && lastPartialDynamicSegmentBytes < FlexRay.DYNAMIC_SEGMENT_SIZE
				&& lastPartialDynamicSegmentBytes >= 0) {

			segmentStartTime = segmentStartTime.plusNanos(this.getStaticSegmentSize().toNanos());
			int remainingBytes = FlexRay.DYNAMIC_SEGMENT_SIZE - this.lastPartialDynamicSegmentBytes;
			int totalBytes = Math.toIntExact((duration.toMillis() * this.mode.getDataRate()) / 8);
			if (totalBytes < remainingBytes) {
				this.fillIncompleteDynamicSegment(segmentStartTime, totalBytes, lastPartialDynamicSegmentBytes);
				this.lastPartialCycleDuration = this.lastPartialCycleDuration.plus(duration);
				this.currentTime = startTime.plus(this.lastPartialCycleDuration);
				this.lastPartialDynamicSegmentBytes += totalBytes;
				this.lastPartialStaticSegmentBytes = 0;
				duration = Duration.ZERO;
			} else {
				this.fillIncompleteDynamicSegment(segmentStartTime, remainingBytes, lastPartialDynamicSegmentBytes);
				this.currentTime = startTime.plus(this.getCycleTime());
				duration = duration.plus(lastPartialCycleDuration).minus(this.getCycleTime());
				this.lastPartialCycleDuration = Duration.ZERO;
				this.lastPartialDynamicSegmentBytes = FlexRay.DYNAMIC_SEGMENT_SIZE;
				this.lastPartialStaticSegmentBytes = this.getTotalStaticSegmentSize();
			}
		}
		return duration;
	}

	/**
	 * Fill a complete static segment
	 * @param segmentStartTime the time at which the segment was started.
	 */
	void fillStaticSegment(Instant segmentStartTime) {
		int transmittedBytes = 0;
		System.out.println("-------------------\n" + "Static segment" + "\n-------------------");
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			transmittedBytes += FlexRay.HEADER_SIZE;
			System.out.println(
					"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
			fillStaticSlot(segmentStartTime, entry.getValue(), transmittedBytes, FlexRay.MAX_SLOT_PAYLOAD, 0);
			transmittedBytes += FlexRay.CONTROLLER_STATIC_PAYLOAD + FlexRay.TRAILER_SIZE;
		}
	}

	/**
	 * Fill an incomplete (it was already started or is not finished) static segment.
	 * @param segmentStartTime the time at which the segment was started
	 * @param bytesToSend the amount of bytes to send (including overhead)
	 * @param startByte the byte from which the segment is filled (including overhead)
	 */
	private void fillIncompleteStaticSegment(Instant segmentStartTime, int bytesToSend, int startByte) {
		int totalTransmittedBytes = startByte;
		System.out.println("-------------------\n" + "Incomplete Static Segment" + "\n-------------------");
		System.out.println("-------------------\n" + "Trasmit " + bytesToSend + " from byte " + startByte
				+ " of " + this.getTotalStaticSegmentSize() + " bytes\n-------------------");
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
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
						System.out.println(
								"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
						this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes,
								startByte);
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
						System.out.println(
								"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
						this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes,
								0);
						bytesToSend -= controllerBytes;
						totalTransmittedBytes += controllerBytes;
						bytesToSend -= FlexRay.TRAILER_SIZE;
						totalTransmittedBytes += FlexRay.TRAILER_SIZE;
					} else {
						bytesToSend -= FlexRay.HEADER_SIZE;
						totalTransmittedBytes += FlexRay.HEADER_SIZE;
						startByte = 0;
						int controllerBytes = Math.min(bytesToSend, FlexRay.MAX_SLOT_PAYLOAD);
						System.out.println(
								"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
						this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes,
								0);
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
	 * @param segmentStartTime the time at which the segment has started
	 * @param msgs the messages that belong to the controller
	 * @param transmittedBytes the bytes that were already transmitted during this segment (including overhead)
	 * @param slotPayload the payload that should be transmitted during the slot
	 * @param startByte the byte from which the slot is filled
	 * @return the unfinished messages of the controller 
	 */
	private PriorityQueue<BusMessage> fillStaticSlot(Instant segmentStartTime, PriorityQueue<BusMessage> msgs,
			int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = ((startByte + slotPayload) % FlexRay.MAX_SLOT_PAYLOAD) == 0;
		while (!msgs.isEmpty() && slotPayload > 0) {
			BusMessage message = msgs.peek();
			if (!message.isTransmitted()) {
				int messageBytes = message.transmitBytes(slotPayload, mode.getBitErrorRate());
				slotPayload -= messageBytes;
				transmittedBytes += messageBytes;
				System.out.println("Msg " + message.getMessage() + ": transmitted: " + messageBytes + " now; "
						+ message.getTransmittedBytes() + " out of " + message.getMessageLen()
						+ "are transmitted in total");
				if (messageBytes >= 0) {
					if (message.isTransmitted()) {
						// remove the message
						msgs.poll();
						if (endOfSlot && slotPayload == 0) {
							transmittedBytes += FlexRay.TRAILER_SIZE;
						}
						long nanoseconds = this.calculateTransmissionTime(transmittedBytes);
						System.out.println("Msg " + message.getMessage() + ": finished after: " + nanoseconds);
						message.setFinishTime(segmentStartTime.plusNanos((long) Math.ceil(nanoseconds)));
						this.registerEventAtSimulator(message);
					}
				} else {
					Log.warn("Error transmitting message in static segment of bus: " + this.getID());
				}
			}
		}
		return msgs;
	}

	/**
	 * Fill a complete dynamic segment
	 * @param segmentStartTime the time at which the segment was started.
	 */
	void fillDynamicSegment(Instant segmentStartTime) {
		System.out.println("-------------------\n" + "Dynamic Segment" + "\n-------------------");
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
	 * Fill an incomplete (it was already started or is not finished) dynamic segment.
	 * @param segmentStartTime the time at which the segment was started
	 * @param bytesToSend the amount of bytes to send (including overhead)
	 * @param startByte the byte from which the segment is filled (including overhead)
	 */
	void fillIncompleteDynamicSegment(Instant segmentStartTime, int bytesToSend, int startByte) {
		int totalTransmittedBytes = startByte;
		System.out.println("-------------------\n" + "Incomplete Dynamic Segment" + "\n-------------------");
		System.out.println("-------------------\n" + "Trasmit " + bytesToSend + " from byte " + startByte
				+ " of " + FlexRay.DYNAMIC_SEGMENT_SIZE + " bytes\n-------------------");
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
		}
		// FlexRay.HeaderSize > transmittedBytes > 0
		else {
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
	 * @param segmentStartTime the time at which the segment has started
	 * @param transmittedBytes the bytes that were already transmitted during this segment (including overhead)
	 * @param slotPayload the payload that should be transmitted during the slot
	 * @param startByte the byte from which the slot is filled
	 */
	private void fillDynamicSlot(Instant segmentStartTime, int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = ((startByte + slotPayload) % FlexRay.MAX_SLOT_PAYLOAD) == 0;
		BusMessage cur = this.getNextDynamicMessage();
		while (cur != null && slotPayload > 0) {
			int messageBytes = cur.transmitBytes(slotPayload, mode.getBitErrorRate());
			slotPayload -= messageBytes;
			transmittedBytes += messageBytes;
			System.out.println("Msg " + cur.getMessage() + ": transmitted: " + messageBytes + "; now "
					+ cur.getTransmittedBytes() + " out of " + cur.getMessageLen() + "are transmitted in total");
			if (messageBytes >= 0) {
				if (cur.isTransmitted()) {
					if (endOfSlot && slotPayload == 0) {
						transmittedBytes += FlexRay.TRAILER_SIZE;
					}
					long nanoseconds = this.calculateTransmissionTime(transmittedBytes);
					cur.setFinishTime(segmentStartTime.plusNanos(nanoseconds));
					this.registerEventAtSimulator(cur);
					System.out.println("Msg " + cur.getMessage() + ": finished after: " + nanoseconds);
					cur = this.getNextDynamicMessage();
				}
			} else {
				Log.warn("Error transmitting message in dynamic segment of bus: " + this.getID());
			}
		}
	}

	/**
	 * @return the message with the highest overall priority
	 */
	BusMessage getNextDynamicMessage() {
		BusMessage res = null;
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
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
		}
		return res;
	}

	/**
	 * Determines if messagesByControllerId contains no messages
	 * 
	 * @param messagesByControllerId the map to check for messages
	 * @return true if messagesByControllerId contains at least one message
	 */
	boolean messagesRegistered() {
		return getNextDynamicMessage() != null;
	}

	long calculateTransmissionTime(long transmittedBytes) {
		long transmittedMikroBits = transmittedBytes * 1000000l * 8l;
		return (long) Math.ceil(transmittedMikroBits / ((double) mode.getDataRate()));
	}
}
