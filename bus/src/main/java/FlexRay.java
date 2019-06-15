
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

	private int connectedComponents;
	
	private Map<String, PriorityQueue<BusMessage>> messagesByControllerId = new HashMap<String, PriorityQueue<BusMessage>>();
	
	public FlexRay(List<String> controllerIds) {
		for(String controllerId : controllerIds) {
			messagesByControllerId.put(controllerId, new PriorityQueue<BusMessage>(COMP_TIME_ASC));
		}
	}

	public FlexRayOperationMode getMode() {
		return mode;
	}

	public void setMode(FlexRayOperationMode mode) {
		this.mode = mode;
	}

	/**
	 * Simulate the transmission of data for a given time
	 * 
	 * @param startTime Time where the simulation starts
	 * @param deltaTime Time of the simulation
	 */

	@Override
	protected void simulateFor(Duration duration) {
		long maxCycle = (duration.get(ChronoUnit.NANOS) / this.getCycleTime().get(ChronoUnit.NANOS));

		// go through simulation cycle by cycle
		Instant cycleEndTime = this.currentTime;
		for (int cycle = 0; cycle <= maxCycle; cycle++) {
			// sent the frames for this cycle
			cycleEndTime = cycleEndTime.plusNanos(this.getCycleTime().get(ChronoUnit.NANOS));
			this.fillStaticSegment(cycleEndTime);
			this.fillDynamicSegment(cycleEndTime);
		}
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
		long nanoseconds = this.getSlotSize().get(ChronoUnit.NANOS) * (this.connectedComponents + DYNAMIC_SLOTS);
		return Duration.ofNanos((long) Math.ceil(nanoseconds));
	}


	/**
	 * @param messagesByControllerId messages indexed by the id of the controller
	 * @param newMessages            messages to be added to
	 *                               {@code messagesByControllerId}
	 * @return {@code messagesByControllerId} with properly inserted messages from
	 *         {@code newMessages}
	 */
	Map<Integer, List<BusMessage>> insertNewMessages(Map<Integer, List<BusMessage>> messagesByControllerId,
			List<BusMessage> newMessages) {
		if (newMessages != null && messagesByControllerId != null) {
			for (BusMessage message : newMessages) {
				List<BusMessage> controllerMessages = messagesByControllerId.getOrDefault(message.getControllerID(),
						new ArrayList<BusMessage>());
				controllerMessages.add(message);
				messagesByControllerId.put(message.getControllerID(), controllerMessages);
			}
		}
		return messagesByControllerId;
	}

	/**
	 * Transmit messages for each controller that wants to send
	 * 
	 * @param messagesByControllerId messages indexed by the id of the controller
	 * @param cycleEndTime           the end time of the current cycle in
	 *                               nanoseconds
	 * @return messages that were successful transmitted during this segment
	 */
	void fillStaticSegment(Instant cycleEndTime) {
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			int transmitted = 0;
			while (!controllerMessages.isEmpty() && transmitted < STATIC_SLOTS * MAX_PAYLOAD_LEN) {
				BusMessage message = controllerMessages.poll();
				if(!message.isTransmitted()) {
					transmitted += message.transmitBytes(STATIC_SLOTS * MAX_PAYLOAD_LEN - transmitted,
							mode.getBitErrorRate());
					if (transmitted >= 0) {
						if (message.isTransmitted()) {
							message.setFinishTime(cycleEndTime);
							this.registerTransmittedEvent(message);
						}
					}
				}
			}
			entry.setValue(controllerMessages);
		}
	}

	/**
	 * @param allMessages  messages that should be transmitted
	 * @param cycleEndTime the end time of the current cycle in nanoseconds
	 * @return messages that were successful transmitted during this segment
	 */
	void fillDynamicSegment(Instant cycleEndTime) {
		int transmitted = 0;
		Optional<BusMessage> cur = this.getNextDynamicMessage();
		while (cur.isPresent() && transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
			transmitted += cur.get().transmitBytes((MAX_PAYLOAD_LEN * DYNAMIC_SLOTS)- transmitted, mode.getBitErrorRate());
			if (transmitted >= 0) {
				if (cur.get().isTransmitted()) {
					cur.get().setFinishTime(cycleEndTime);
					this.registerTransmittedEvent(cur.get());
				}
			}
		}
	}

	private Optional<BusMessage> getNextDynamicMessage() {
		Optional<BusMessage> res = Optional.ofNullable(null);
		for (Map.Entry<String, PriorityQueue<BusMessage>> entry : messagesByControllerId.entrySet()) {
			PriorityQueue<BusMessage> controllerMessages = entry.getValue();
			if(!controllerMessages.isEmpty()) {
				BusMessage msg = controllerMessages.peek();
				if(!msg.isTransmitted()) {
					if(!res.isPresent() || COMP_TIME_ASC.compare(res.get(), msg) > 0) {
						res = Optional.of(msg);
					}
				}
				else {
					controllerMessages.poll();
				}
			}
		}
		return res;
	}

	@Override
	protected void removeKeepAlive() {
		// TODO Auto-generated method stub	
	}

	@Override
	protected void setKeepAlive() {
		// TODO Auto-generated method stub
		
	}

	@Override
	protected void registerMessage(BusMessage msg) {
		// TODO Auto-generated method stub
		
	}

}
