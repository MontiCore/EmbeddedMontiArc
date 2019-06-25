
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
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.stream.Collectors;

import commons.simulation.DiscreteEvent;

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

	/**
	 * Comparator that sorts {@link BusMessages} ascending according to their id
	 */
	private static final BusMessageComparatorIdDesc COMP_ID_DESC = new BusMessageComparatorIdDesc();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

	private Map<String, PriorityQueue<BusMessage>> messagesByControllerId = new HashMap<String, PriorityQueue<BusMessage>>();

	public FlexRay(EESimulator simulator, List<EEComponent> controllers) {
		super(simulator, controllers);
		for (EEComponent controller : controllers) {
			messagesByControllerId.put(controller.getID().toString(), new PriorityQueue<BusMessage>(COMP_ID_DESC));
		}
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

	@Override
	protected void simulateFor(Duration duration) {
		long maxCycle = (duration.toNanos() / this.getCycleTime().toNanos());

		// go through simulation cycle by cycle
		Instant cycleEndTime = this.currentTime;
		Duration cylceTime = this.getCycleTime();
		for (int cycle = 0; cycle <= maxCycle && this.messagesRegistered(); cycle++) {
			// sent the frames for this cycle
			cycleEndTime = cycleEndTime.plusNanos(cylceTime.toNanos());
			this.fillStaticSegment(cycleEndTime);
			this.fillDynamicSegment(cycleEndTime);
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
			minCycle = (int) Math.ceil(toTransmit / (MAX_PAYLOAD_LEN * (DYNAMIC_SLOTS + 1)));
			while (!firstMsgs.isEmpty()) {
				BusMessage cur = firstMsgs.poll();
				// transmitted during static segments
				int msgCycles = (int) Math.ceil(cur.getRemainingBytes() / ((double) MAX_PAYLOAD_LEN));
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
		double nanoseconds = ((HEADER_SIZE + TRAILER_SIZE + MAX_PAYLOAD_LEN) * 8 * 1000) / (double) mode.getDataRate();
		return Duration.ofNanos((long) Math.ceil(nanoseconds));
	}

	/**
	 * @return The duration of a cycle
	 */
	Duration getCycleTime() {
		long nanoseconds = this.getSlotSize().toNanos() * (this.messagesByControllerId.size() + DYNAMIC_SLOTS);
		return Duration.ofNanos((long) Math.ceil(nanoseconds));
	}

	/**
	 * Transmit messages for each controller that wants to send
	 * 
	 * @param messagesByControllerId the messages that should be used to fill the
	 *                               segment
	 * @param cycleEndTime           the end time of the current cycle
	 * @param actual                 true if this is called within an actual
	 *                               simulation. False otherwise (for example when
	 *                               called to calculate the finish time of the next
	 *                               message)
	 * @return true if at least one message was finished in the static segment
	 */
	boolean fillStaticSegment(Instant cycleEndTime) {
		boolean res = false;
		System.out.println("-------------------\n" + "Static segment" + "\n-------------------");
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			int transmitted = 0;
			while (!controllerMessages.isEmpty() && transmitted < STATIC_SLOTS * MAX_PAYLOAD_LEN) {
				System.out.println("-------------------\n" + "Controller " + entry.getKey() + "\n-------------------");
				BusMessage message = controllerMessages.peek();
				if (!message.isTransmitted()) {
					transmitted += message.transmitBytes((STATIC_SLOTS * MAX_PAYLOAD_LEN) - transmitted,
							mode.getBitErrorRate());
					System.out.println("Msg " + message.getMessage() + ": total amount transmitted: " + message.getTransmittedBytes()
					+ " out of " + message.getMessageLen());
					if (transmitted >= 0) {
						if (message.isTransmitted()) {
							// remove the message
							controllerMessages.poll();
							res = true;
							message.setFinishTime(cycleEndTime);
							this.registerEventAtSimulator(message);
						}
					}
				}
			}
			entry.setValue(controllerMessages);
		}
		return res;
	}

	/**
	 * Fill the dynamic segment with the messages with highest overall priority
	 * 
	 * @param messagesByControllerId the messages that should be used to fill the
	 *                               segment
	 * @param cycleEndTime           the end time of the current cycle
	 * @param actual                 true if this is called within an actual
	 *                               simulation. False otherwise (for example when
	 *                               called to calculate the finish time of the next
	 *                               message)
	 * @return true if at least one message was finished in the dynamic segment
	 */
	boolean fillDynamicSegment(Instant cycleEndTime) {
		boolean res = false;
		System.out.println("-------------------\n" + "Dynamic segment" + "\n-------------------");
		int transmitted = 0;
		BusMessage cur = this.getNextDynamicMessage();
		while (cur != null && transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
			transmitted += cur.transmitBytes((MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) - transmitted, mode.getBitErrorRate());
			System.out.println("Msg " + cur.getMessage() + ": total amount transmitted: " + cur.getTransmittedBytes()
					+ " out of " + cur.getMessageLen());
			if (transmitted >= 0) {
				if (cur.isTransmitted()) {
					res = true;
					cur.setFinishTime(cycleEndTime);
					this.registerEventAtSimulator(cur);
					cur = this.getNextDynamicMessage();
				}
			}
		}
		return res;
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
}
