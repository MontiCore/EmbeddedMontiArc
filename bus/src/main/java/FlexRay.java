
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
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

public class FlexRay implements Bus {

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
	private static final BusMessageComparator COMP_ASC_ID = new BusMessageComparator();

	/**
	 * Number of connected components
	 */
	private int connectedComponents;

	/**
	 * The messages that should be transmitted by this bus
	 */
	private List<BusMessage> activeMessages = new ArrayList<BusMessage>();

	/**
	 * Map with all transmitted messages
	 */
	public Map<String, BusMessage> transmittedMessages = new HashMap<String, BusMessage>();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

	public FlexRay(List<Object> connectedComponents) {
		if(connectedComponents != null) {
			this.connectedComponents = connectedComponents.size();
		}
		else {
			throw new IllegalArgumentException("connectedComponents can not be null");
		}
	}
	
	public FlexRayOperationMode getMode() {
		return mode;
	}

	public void setMode(FlexRayOperationMode mode) {
		this.mode = mode;
	}

	@Override
	public void registerData(String key, BusMessage msg) {
		// TODO Auto-generated method stub
		this.activeMessages.add(msg);
	}

	@Override
	public Optional<BusMessage> getData(String key) {
		// TODO Auto-generated method stub
		return Optional.of(transmittedMessages.get(key));
	}

	@Override
	public Map<String, BusMessage> getAllData() {
		// TODO Auto-generated method stub
		return transmittedMessages;
	}

	@Override
	public String[] getImportNames() {
		// TODO Auto-generated method stub
		Set keys = transmittedMessages.keySet();
		return (String[]) keys.toArray();
	}
	
	@Override
	public void registerComponent(Object component) {
		// TODO Auto-generated method stub
	}

	/**
	 * @param deltaTime time until the next frame will be transmitted
	 * @return time that message needs until it is transmitted
	 */
	public int getDelay(int deltaTime) {
		int delay = deltaTime + getCycleTime();
		return delay;
	}

	/**
	 * Simulate the transmission of data for a given time
	 * 
	 * @param startTime Time where the simulation starts
	 * @param deltaTime Time of the simulation
	 */
	@Override
	public List<BusMessage> simulateFor(int startTime, int duration) {
		List<BusMessage> finishedMessages = new ArrayList<BusMessage>();
		// index messages by start cycle
		Map<Integer, List<BusMessage>> messagesByStartCycle = calculateStartCycles();
		int maxStartCycle = Collections.max(messagesByStartCycle.keySet());
		int maxCycle = (duration / this.getCycleTime());

		// go through simulation cycle by cycle
		Map<Integer, List<BusMessage>> messagesByControllerId = new HashMap<Integer, List<BusMessage>>();
		for (int cycle = 0; cycle <= maxCycle
				&& (cycle <= maxStartCycle || !messagesByControllerId.isEmpty()); cycle++) {
			// add new messages in that arrived in this cycle (if any)
			List<BusMessage> cycleMessages = messagesByStartCycle.remove(cycle);
			messagesByControllerId = this.insertNewMessages(messagesByControllerId, cycleMessages);

			// sent the frames for this cycle
			int cycleEndTime = startTime + (cycle + 1) * this.getCycleTime();
			List<BusMessage> res = this.fillStaticSegment(messagesByControllerId, cycleEndTime);
			// remove messages that are successfully transmitted from map
			for(BusMessage finishedMsg : res) {
				List<BusMessage> controllerMsgs = messagesByControllerId.remove(finishedMsg.getControllerID());
				controllerMsgs.remove(finishedMsg);
				if(!controllerMsgs.isEmpty()) {
					messagesByControllerId.put(finishedMsg.getControllerID(), controllerMsgs);
				}
			}
			finishedMessages.addAll(res); 
			List<BusMessage> allMessages = messagesByControllerId.values().stream().flatMap(List::stream)
					.collect(Collectors.toList());
			res = this.fillDynamicSegment(allMessages, cycleEndTime);
			finishedMessages.addAll(res); 

			// remove messages that are successfully transmitted from map
			for(BusMessage finishedMsg : res) {
				List<BusMessage> controllerMsgs = messagesByControllerId.remove(finishedMsg.getControllerID());
				controllerMsgs.remove(finishedMsg);
				if(!controllerMsgs.isEmpty()) {
					messagesByControllerId.put(finishedMsg.getControllerID(), controllerMsgs);
				}
			}
		}
		for(BusMessage msg : finishedMessages) {
			activeMessages.remove(msg);
		}
		return finishedMessages;
	}
	
	/**
	 * @return The duration of a slot in microseconds
	 */
	int getSlotSize() {
		return (int) Math.ceil(((HEADER_SIZE + TRAILER_SIZE + MAX_PAYLOAD_LEN) * 8) / (double) mode.getDataRate());
	}

	/**
	 * @return The duration of a cycle in microseconds
	 */
	int getCycleTime() {
		return this.getSlotSize() * (this.connectedComponents + DYNAMIC_SLOTS);
	}

	/**
	 * @return Map with cycle number as key and a list of messages that start in
	 *         this cycle as value
	 */
	Map<Integer, List<BusMessage>> calculateStartCycles() {
		Map<Integer, List<BusMessage>> messagesByStartCycle = new HashMap<Integer, List<BusMessage>>();
		for (BusMessage message : activeMessages) {
			int startCycle = message.getRequestTime() / getCycleTime();
			List<BusMessage> startCycleMessages = messagesByStartCycle.getOrDefault(startCycle,
					new ArrayList<BusMessage>());
			startCycleMessages.add(message);
			messagesByStartCycle.put(startCycle, startCycleMessages);
		}
		return messagesByStartCycle;
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
	 * @param cycleEndTime           the end time of the current cycle
	 * @return messages that were successful transmitted during this segment
	 */
	List<BusMessage> fillStaticSegment(Map<Integer, List<BusMessage>> messagesByControllerId, int cycleEndTime) {
		List<BusMessage> finishedMessages = new ArrayList<BusMessage>();
		for (Map.Entry<Integer, List<BusMessage>> entry : messagesByControllerId.entrySet()) {
			List<BusMessage> controllerMessages = entry.getValue();
			int transmitted = 0;
			while (!controllerMessages.isEmpty() && transmitted < STATIC_SLOTS * MAX_PAYLOAD_LEN) {
				controllerMessages.sort(COMP_ASC_ID);
				BusMessage message = controllerMessages.get(0);
				transmitted += message.transmitBytes(STATIC_SLOTS * MAX_PAYLOAD_LEN - transmitted,
						mode.getBitErrorRate());
				if (transmitted >= 0) {
					if (message.isTransmitted()) {
						controllerMessages.remove(message);
						finishedMessages.add(message);
						message.setFinishTime(cycleEndTime);
					}
				}
			}
			entry.setValue(controllerMessages);
		}
		return finishedMessages;
	}

	/**
	 * @param allMessages  messages that should be transmitted
	 * @param cycleEndTime the end time of the current cycle
	 * @return messages that were successful transmitted during this segment
	 */
	List<BusMessage> fillDynamicSegment(List<BusMessage> allMessages, int cycleEndTime) {
		List<BusMessage> finishedMessages = new ArrayList<BusMessage>();
		int transmitted = 0;
		while (!allMessages.isEmpty() && transmitted < MAX_PAYLOAD_LEN * DYNAMIC_SLOTS) {
			allMessages.sort(new BusMessageComparator());
			BusMessage message = allMessages.get(0);
			transmitted += message.transmitBytes(MAX_PAYLOAD_LEN - transmitted, mode.getBitErrorRate());
			if (transmitted >= 0) {
				if (message.isTransmitted()) {
					allMessages.remove(message);
					finishedMessages.add(message);
					message.setFinishTime(cycleEndTime);
				}
			}
		}
		return allMessages;
	}

}
