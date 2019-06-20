
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
	private static final int MAX_PAYLOAD_LEN = 254;

	/**
	 * Number of static slots per station
	 */
	private static final int STATIC_SLOTS = 1;

	/**
	 * Number of dynamic slots per cycle
	 */
	private static final int DYNAMIC_SLOTS = 4;

	/**
	 * Comparator that sorts {@link BusMessages} ascending according to their id
	 */
	private static final BusMessageComparatorIdAsc COMP_ASC_ID = new BusMessageComparatorIdAsc();

	private static final BusMessageComparatorTimeAsc COMP_TIME_ASC = new BusMessageComparatorTimeAsc();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

	private Map<String, PriorityQueue<BusMessage>> messagesByControllerId = new HashMap<String, PriorityQueue<BusMessage>>();

	public FlexRay(EESimulator simulator, List<EEComponent> controllers) {
		super(simulator, controllers);
		for (EEComponent controller : controllers) {
			messagesByControllerId.put(controller.getID().toString(), new PriorityQueue<BusMessage>(COMP_TIME_ASC));
		}
	}

	public FlexRayOperationMode getMode() {
		return mode;
	}

	public void setMode(FlexRayOperationMode mode) {
		this.mode = mode;
	}

	@Override
	protected void simulateFor(Duration duration) {
		long maxCycle = (duration.toNanos() / this.getCycleTime().toNanos());

		// go through simulation cycle by cycle
		Instant cycleEndTime = this.currentTime;
		Duration cylceTime = this.getCycleTime();
		// TODO take care of unnecessary cycles (no messages to send)
		for (int cycle = 0; cycle <= maxCycle; cycle++) {
			// sent the frames for this cycle
			cycleEndTime = cycleEndTime.plusNanos(cylceTime.toNanos());
			this.fillStaticSegment(cycleEndTime, true);
			this.fillDynamicSegment(cycleEndTime, true);
		}
	}

	@Override
	protected void registerMessage(BusMessage msg) {
		if (messagesByControllerId.containsKey(msg.getMessageID().toString())) {
			PriorityQueue<BusMessage> controllerMsgs = messagesByControllerId.get(msg.getMessageID().toString());
			controllerMsgs.add(msg);
			messagesByControllerId.put(msg.getMessageID().toString(), controllerMsgs);
		} else {
			throw new IllegalArgumentException("Message does not belong to a registered controller.");
		}
	}

	@Override
	protected Instant getNextFinishTime() {
		Instant nextFinishTime = this.currentTime;
		Duration cylceTime = this.getCycleTime();
		boolean transmittedMessage = false;
		// TODO take care of unnecessary cycles (no messages to send)
		while (!transmittedMessage) {
			// sent the frames for this cycle
			nextFinishTime = nextFinishTime.plusNanos(cylceTime.toNanos());
			transmittedMessage = this.fillStaticSegment(nextFinishTime, false);
			transmittedMessage = this.fillDynamicSegment(nextFinishTime, false);
		}
		return nextFinishTime;
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
	 * @param cycleEndTime the end time of the current cycle
	 * @param actual       true if this is called within an actual simulation. False
	 *                     otherwise (for example when called to calculate the
	 *                     finish time of the next message)
	 * @return true if at least one message was finished in the static segment
	 */
	boolean fillStaticSegment(Instant cycleEndTime, boolean actual) {
		boolean res = false;
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			int transmitted = 0;
			while (!controllerMessages.isEmpty() && transmitted < STATIC_SLOTS * MAX_PAYLOAD_LEN) {
				BusMessage message = controllerMessages.poll();
				if (!message.isTransmitted()) {
					// TODO transmitBytes may update bytes incorrectly
					transmitted += message.transmitBytes(STATIC_SLOTS * MAX_PAYLOAD_LEN - transmitted,
							mode.getBitErrorRate());
					if (transmitted >= 0) {
						if (message.isTransmitted()) {
							res = true;
							if(actual) {
								message.setFinishTime(cycleEndTime);
								this.registerEvent(message);
							}
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
	 * @param cycleEndTime the end time of the current cycle
	 * @param actual       true if this is called within an actual simulation. False
	 *                     otherwise (for example when called to calculate the
	 *                     finish time of the next message)
	 * @return true if at least one message was finished in the dynamic segment
	 */
	boolean fillDynamicSegment(Instant cycleEndTime, boolean actual) {
		boolean res = false;
		int transmitted = 0;
		Optional<BusMessage> cur = this.getNextDynamicMessage();
		while (cur.isPresent() && transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
			transmitted += cur.get().transmitBytes((MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) - transmitted,
					mode.getBitErrorRate());
			if (transmitted >= 0) {
				if (cur.get().isTransmitted()) {
					res = true;
					if(actual) {
						cur.get().setFinishTime(cycleEndTime);
						this.registerEvent(cur.get());
					}
				}
			}
		}
		return res;
	}

	/**
	 * @return the message with the highest overall priority
	 */
	Optional<BusMessage> getNextDynamicMessage() {
		Optional<BusMessage> res = Optional.ofNullable(null);
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			if (!controllerMessages.isEmpty()) {
				BusMessage msg = controllerMessages.peek();
				if (!msg.isTransmitted()) {
					if (!res.isPresent() || COMP_TIME_ASC.compare(res.get(), msg) > 0) {
						res = Optional.of(msg);
					}
				} else {
					controllerMessages.poll();
				}
			}
		}
		return res;
	}
}
