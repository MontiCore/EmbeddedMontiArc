
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
import commons.controller.commons.BusEntry;

public class FlexRay extends Bus {

	/**
	 * Size of frame header in byte
	 */
	private static final int HEADER_SIZE = 5;

	/**
	 * Size of frame trailer in byte
	 */
	private static final int TRAILER_SIZE = 3;

	/**
	 * Maximum bytes of payload that fit in one frame
	 */
	protected static final int MAX_PAYLOAD_LEN = 254;

	/**
	 * Number of static slots per station
	 */
	private static final int STATIC_SLOTS = 1;

	/**
	 * Number of dynamic slots per cycle
	 */
	protected static final int DYNAMIC_SLOTS = 4;

	protected static int getControllerStaticPayload() {
		return FlexRay.STATIC_SLOTS * FlexRay.MAX_PAYLOAD_LEN;
	}

	protected static int getControllerSlotSize() {
		return (FlexRay.STATIC_SLOTS * FlexRay.MAX_PAYLOAD_LEN) + FlexRay.HEADER_SIZE + FlexRay.TRAILER_SIZE;
	}

	protected static int getTotalDynamicPayload() {
		return FlexRay.DYNAMIC_SLOTS * FlexRay.MAX_PAYLOAD_LEN;
	}

	protected static int getTotalDynamicSegmentSize() {
		return FlexRay.DYNAMIC_SLOTS * (FlexRay.HEADER_SIZE + FlexRay.MAX_PAYLOAD_LEN + FlexRay.TRAILER_SIZE);
	}

	/**
	 * Comparator that sorts {@link BusMessages} ascending according to their id
	 */
	private static final BusMessageComparatorIdDesc COMP_ID_DESC = new BusMessageComparatorIdDesc();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

	private Map<String, PriorityQueue<BusMessage>> messagesByControllerId = new LinkedHashMap<String, PriorityQueue<BusMessage>>();

	private Duration lastPartialCycleDuration = Duration.ZERO;

	private int lastPartialStaticSegmentBytes = 0;

	private int lastPartialDynamicSegmentBytes = 0;

	public FlexRay(EESimulator simulator) {
		super(simulator);
	}

	public void registerComponent(EEComponent component, List<BusEntry> messages){
		super.registerComponent(component, messages);
		messagesByControllerId.put(component.getID().toString(), new PriorityQueue<BusMessage>(COMP_ID_DESC));
	}

	int getTotalStaticPayload() {
		return FlexRay.getControllerStaticPayload() * this.connectedComponents.size();
	}

	int getTotalStaticSegmentSize() {
		return (FlexRay.HEADER_SIZE + FlexRay.getControllerStaticPayload() + FlexRay.TRAILER_SIZE)
				* this.connectedComponents.size();
	}

	public FlexRayOperationMode getMode() {
		return mode;
	}

	public void setMode(FlexRayOperationMode mode) {
		this.mode = mode;
	}

	Map<String, PriorityQueue<BusMessage>> getMessagesByControllerId() {
		return this.messagesByControllerId;
	}

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
				remainingBytes = FlexRay.getTotalDynamicSegmentSize();
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
				}
			}
		} else if (lastPartialStaticSegmentBytes > 0
				&& lastPartialDynamicSegmentBytes < FlexRay.getTotalDynamicPayload()
				&& lastPartialDynamicSegmentBytes >= 0) {

			segmentStartTime.plusNanos(this.getStaticSegmentSize().toNanos());
			int remainingBytes = FlexRay.getTotalDynamicSegmentSize() - this.lastPartialDynamicSegmentBytes;
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
			}
		}
		return duration;
	}

	@Override
	protected void simulateFor(Duration duration) {

		duration = this.finishLastCycle(duration);

		if (!duration.isZero()) {
			Instant segmentStartTime = this.currentTime.minusNanos(lastPartialCycleDuration.toNanos());
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
		}

	}

//	@Override
//	protected void registerMessage(BusMessage msg) {
//		if (msg.getPath().isEmpty()) {
//			boolean suc = this.setPath(msg);
//			if (!suc) {
//				throw new IllegalArgumentException("Message send to unknown controller.");
//			}
//		}
//		if (!messagesByControllerId.containsKey(msg.getControllerID())) {
//			throw new IllegalArgumentException("Message send by unknown controller.");
//		} else {
//			PriorityQueue<BusMessage> controllerMsgs = messagesByControllerId.get(msg.getControllerID());
//			controllerMsgs.add(msg);
//			messagesByControllerId.put(msg.getControllerID(), controllerMsgs);
//		}
//	}

	@Override
	protected void registerMessage(BusMessage msg){
		if(!sendTo.containsKey(msg.getMessageID())){
			throw new IllegalArgumentException("Message has no target in this Bus.");
		}
		if(!messagesByControllerId.containsKey(msg.getControllerID())){
			throw new IllegalArgumentException("Message send by unknown controller.");
		}
		else{
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
			minCycle = (int) Math.ceil(toTransmit / (MAX_PAYLOAD_LEN * (DYNAMIC_SLOTS + STATIC_SLOTS)));
			while (!firstMsgs.isEmpty()) {
				BusMessage cur = firstMsgs.poll();
				// transmitted during static segments
				int msgCycles = (int) Math
						.ceil(cur.getRemainingBytes() / ((double) FlexRay.getControllerStaticPayload()));
				if (msgCycles < minCycle) {
					minCycle = msgCycles;
				}
			}
		}

		return this.currentTime.plusNanos(this.getCycleTime().toNanos() * minCycle);
	}

	/**
	 * @return The duration of a slot
	 */
	Duration getSlotSize() {
		double nanoseconds = (((long) (HEADER_SIZE + TRAILER_SIZE + MAX_PAYLOAD_LEN)) * 8l * 1000000l)
				/ ((double) mode.getDataRate());
		return Duration.ofNanos((long) Math.ceil(nanoseconds));
	}

	/**
	 * @return The duration of the static segment
	 */
	Duration getStaticSegmentSize() {
		long nanoseconds = this.getSlotSize().toNanos() * this.messagesByControllerId.size();
		return Duration.ofNanos(nanoseconds);
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
		long nanoseconds = this.getSlotSize().toNanos() * (this.messagesByControllerId.size() + FlexRay.DYNAMIC_SLOTS);
		return Duration.ofNanos(nanoseconds);
	}

	void fillStaticSegment(Instant segmentStartTime) {
		int transmittedBytes = 0;
		System.out.println("-------------------\n" + "Static segment" + "\n-------------------");
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			transmittedBytes += FlexRay.HEADER_SIZE;
			fillStaticSlot(segmentStartTime, entry.getValue(), transmittedBytes, FlexRay.getControllerStaticPayload(), 0);
			transmittedBytes += FlexRay.getControllerStaticPayload() + FlexRay.TRAILER_SIZE;
		}
	}

	void fillIncompleteStaticSegment(Instant segmentStartTime, int bytesToSend, int transmittedBytes) {
		int totalTransmittedBytes = transmittedBytes;
		System.out.println("-------------------\n" + "Incomplete Static Segment" + "\n-------------------");
		System.out.println("-------------------\n" + "Trasmit " + bytesToSend + " from byte " + transmittedBytes
				+ " of " + this.getTotalStaticSegmentSize() + " bytes\n-------------------");
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			if (bytesToSend > 0) {
				if (transmittedBytes > FlexRay.getControllerSlotSize()) {
					transmittedBytes -= FlexRay.HEADER_SIZE;
					transmittedBytes -= FlexRay.getControllerStaticPayload();
					transmittedBytes -= FlexRay.TRAILER_SIZE;
				} else if (transmittedBytes > (FlexRay.HEADER_SIZE + FlexRay.getControllerStaticPayload())) {
					transmittedBytes -= FlexRay.HEADER_SIZE;
					transmittedBytes -= FlexRay.getControllerStaticPayload();
					bytesToSend -= (FlexRay.TRAILER_SIZE - transmittedBytes);
					totalTransmittedBytes += (FlexRay.TRAILER_SIZE - transmittedBytes);
					transmittedBytes = 0;
				} else if (transmittedBytes > FlexRay.HEADER_SIZE) {
					transmittedBytes -= FlexRay.HEADER_SIZE;
					int controllerBytes = Math.max(bytesToSend,
							FlexRay.getControllerStaticPayload() - transmittedBytes);
					System.out.println(
							"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
					this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes, transmittedBytes);
					transmittedBytes = 0;
					bytesToSend -= controllerBytes;
					totalTransmittedBytes += controllerBytes;
					bytesToSend -= FlexRay.TRAILER_SIZE;
					totalTransmittedBytes += FlexRay.TRAILER_SIZE;
				} else if (transmittedBytes > 0) {
					bytesToSend -= FlexRay.HEADER_SIZE - transmittedBytes;
					totalTransmittedBytes += FlexRay.HEADER_SIZE - transmittedBytes;
					transmittedBytes = 0;
					int controllerBytes = Math.min(bytesToSend, FlexRay.getControllerStaticPayload());
					System.out.println(
							"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
					this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes, 0);
					bytesToSend -= controllerBytes;
					totalTransmittedBytes += controllerBytes;
					bytesToSend -= FlexRay.TRAILER_SIZE;
					totalTransmittedBytes += FlexRay.TRAILER_SIZE;
				} else {
					bytesToSend -= FlexRay.HEADER_SIZE;
					totalTransmittedBytes += FlexRay.HEADER_SIZE;
					transmittedBytes = 0;
					int controllerBytes = Math.min(bytesToSend, FlexRay.getControllerStaticPayload());
					System.out.println(
							"-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
					this.fillStaticSlot(segmentStartTime, entry.getValue(), totalTransmittedBytes, controllerBytes, 0);
					bytesToSend -= controllerBytes;
					totalTransmittedBytes += controllerBytes;
					bytesToSend -= FlexRay.TRAILER_SIZE;
					totalTransmittedBytes += FlexRay.TRAILER_SIZE;
				}
			}
		}
	}

	PriorityQueue<BusMessage> fillStaticSlot(Instant segmentStartTime, PriorityQueue<BusMessage> msgs,
			int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = ((startByte + slotPayload) == FlexRay.getControllerStaticPayload());
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

	void fillDynamicSegment(Instant segmentStartTime) {
		System.out.println("-------------------\n" + "Dynamic Segment" + "\n-------------------");
		int bytesToSend = FlexRay.getTotalDynamicSegmentSize();
		int transmittedBytes = 0;
		while (bytesToSend > 0) {
			bytesToSend -= FlexRay.HEADER_SIZE;
			transmittedBytes += FlexRay.HEADER_SIZE;
			int slotPayload = Math.min(FlexRay.getControllerStaticPayload(), bytesToSend);
			this.fillDynamicSlot(segmentStartTime, transmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			transmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}
	}

	void fillIncompleteDynamicSegment(Instant segmentStartTime, int bytesToSend, int transmittedBytes) {
		int totalTransmittedBytes = transmittedBytes;
		System.out.println("-------------------\n" + "Incomplete Dynamic Segment" + "\n-------------------");
		System.out.println("-------------------\n" + "Trasmit " + bytesToSend + " from byte " + transmittedBytes
				+ " of " + FlexRay.getTotalDynamicPayload() + " bytes\n-------------------");
		int fullSlots = transmittedBytes / FlexRay.getControllerSlotSize();
		transmittedBytes -= (fullSlots * FlexRay.getControllerSlotSize());
		if (transmittedBytes > FlexRay.HEADER_SIZE + FlexRay.getControllerStaticPayload()) {
			transmittedBytes -= FlexRay.HEADER_SIZE + FlexRay.getControllerStaticPayload();
			bytesToSend -= FlexRay.TRAILER_SIZE - transmittedBytes;
			totalTransmittedBytes += FlexRay.TRAILER_SIZE - transmittedBytes;
		} else if (transmittedBytes > FlexRay.HEADER_SIZE) {
			transmittedBytes -= FlexRay.HEADER_SIZE;
			int slotPayload = Math.min(FlexRay.getControllerStaticPayload() - transmittedBytes, bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, transmittedBytes);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE - transmittedBytes;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE - transmittedBytes;
		}
		// FlexRay.HeaderSize > transmittedBytes > 0
		else {
			bytesToSend -= FlexRay.HEADER_SIZE - transmittedBytes;
			totalTransmittedBytes += FlexRay.HEADER_SIZE - transmittedBytes;
			int slotPayload = Math.min(FlexRay.getControllerStaticPayload(), bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}

		while (bytesToSend > 0) {
			bytesToSend -= FlexRay.HEADER_SIZE;
			totalTransmittedBytes += FlexRay.HEADER_SIZE;
			int slotPayload = Math.min(FlexRay.getControllerStaticPayload(), bytesToSend);
			this.fillDynamicSlot(segmentStartTime, totalTransmittedBytes, slotPayload, 0);
			bytesToSend -= slotPayload + FlexRay.TRAILER_SIZE;
			totalTransmittedBytes += slotPayload + FlexRay.TRAILER_SIZE;
		}
	}

	private void fillDynamicSlot(Instant segmentStartTime, int transmittedBytes, int slotPayload, int startByte) {
		boolean endOfSlot = (startByte + slotPayload) == FlexRay.getControllerSlotSize();
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
					cur.setFinishTime(segmentStartTime.plusNanos((long) Math.ceil(nanoseconds)));
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
