
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
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.stream.Collectors;

import commons.simulation.DiscreteEvent;
import commons.simulation.SimulationLoopExecutable;

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
	private static final BusMessageComparator COMP_ASC_ID = new BusMessageComparator();

	/**
	 * The mode in which this bus is configured {@link FlexRayOpterationMode}
	 */
	private FlexRayOperationMode mode = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);

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
	public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {
		List<BusMessage> finishedMessages = new ArrayList<BusMessage>();
		long maxCycle = (deltaTime / this.getCycleTime());

		// go through simulation cycle by cycle
		Map<Integer, List<BusMessage>> messagesByControllerId = new HashMap<Integer, List<BusMessage>>();
		for (int cycle = 0; cycle <= maxCycle; cycle++) {
			// add new messages in that arrived in this cycle (if any)
			List<BusMessage> cycleMessages = this.getIncomingMessagesUnitl(cycle * this.getCycleTime() + totalTime);
			messagesByControllerId = this.insertNewMessages(messagesByControllerId, cycleMessages);

			// sent the frames for this cycle
			long cycleEndTime = totalTime + (cycle + 1) * this.getCycleTime();
			List<BusMessage> res = this.fillStaticSegment(messagesByControllerId, cycleEndTime);
			// remove messages that are successfully transmitted from map
			for (BusMessage finishedMsg : res) {
				List<BusMessage> controllerMsgs = messagesByControllerId.remove(finishedMsg.getControllerID());
				controllerMsgs.remove(finishedMsg);
				if (!controllerMsgs.isEmpty()) {
					messagesByControllerId.put(finishedMsg.getControllerID(), controllerMsgs);
				}
			}
			finishedMessages.addAll(res);
			List<BusMessage> allMessages = messagesByControllerId.values().stream().flatMap(List::stream)
					.collect(Collectors.toList());
			res = this.fillDynamicSegment(allMessages, cycleEndTime);
			finishedMessages.addAll(res);

			// remove messages that are successfully transmitted from map
			for (BusMessage finishedMsg : res) {
				List<BusMessage> controllerMsgs = messagesByControllerId.remove(finishedMsg.getControllerID());
				controllerMsgs.remove(finishedMsg);
				if (!controllerMsgs.isEmpty()) {
					messagesByControllerId.put(finishedMsg.getControllerID(), controllerMsgs);
				}
			}
		}
		for (BusMessage msg : finishedMessages) {
			activeMessages.remove(msg);
			this.scheduleEvent(new BusMessageDeliveredEvent(msg));
		}
		super.didExecuteLoop(simulationObjects, totalTime, deltaTime);
	}

	/**
	 * @return The duration of a slot in nanoseconds
	 */
	long getSlotSize() {
		return (long) Math
				.ceil(((HEADER_SIZE + TRAILER_SIZE + MAX_PAYLOAD_LEN) * 8 * 1000) / (double) mode.getDataRate());
	}

	/**
	 * @return The duration of a cycle in nanoseconds
	 */
	long getCycleTime() {
		return this.getSlotSize() * (this.connectedComponents + DYNAMIC_SLOTS);
	}

	/**
	 * @param finalTime time in nanoseconds
	 * @return all incoming messages until time finalTime
	 */
	List<BusMessage> getIncomingMessagesUnitl(long finalTime) {
		List<BusMessage> res = new ArrayList<BusMessage>();
		List<DiscreteEvent> events = this.getEventList();
		Optional<BusMessageTransmissionRequestEvent> transmissionReq = Optional.empty();
		if (!events.isEmpty()) {
			DiscreteEvent event = events.get(0);
			if (event.getEventTime() <= finalTime && event instanceof BusMessageTransmissionRequestEvent) {
				transmissionReq = Optional.of((BusMessageTransmissionRequestEvent) event);
				activeMessages.add(transmissionReq.get().getMessage());
				res.add(transmissionReq.get().getMessage());
			} else {
				transmissionReq = Optional.empty();
				if (!(event instanceof BusMessageTransmissionRequestEvent)) {
					// TODO: error!
				}
			}
		}
		while (!events.isEmpty() && transmissionReq.isPresent()) {
			DiscreteEvent event = events.get(0);
			if (event.getEventTime() <= finalTime && event instanceof BusMessageTransmissionRequestEvent) {
				transmissionReq = Optional.of((BusMessageTransmissionRequestEvent) event);
				activeMessages.add(transmissionReq.get().getMessage());
				res.add(transmissionReq.get().getMessage());
			} else {
				transmissionReq = Optional.empty();
				if (!(event instanceof BusMessageTransmissionRequestEvent)) {
					// TODO: error!
				}
			}
		}
		this.setEventList(events);
		return res;
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
	List<BusMessage> fillStaticSegment(Map<Integer, List<BusMessage>> messagesByControllerId, long cycleEndTime) {
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
	 * @param cycleEndTime the end time of the current cycle in nanoseconds
	 * @return messages that were successful transmitted during this segment
	 */
	List<BusMessage> fillDynamicSegment(List<BusMessage> allMessages, long cycleEndTime) {
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
