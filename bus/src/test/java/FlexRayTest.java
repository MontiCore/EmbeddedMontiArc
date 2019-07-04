
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
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Random;

import static commons.controller.commons.BusEntry.*;

import bus.*;

import org.junit.BeforeClass;
import org.junit.Test;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.MessageType;
import simulation.bus.*;

public class FlexRayTest {

	private static EESimulator EEsim = new EESimulator(Instant.EPOCH);

	private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

	@BeforeClass
	public static void oneTimeSetUp() {
		for (BusEntry entry : BusEntry.values()) {
			busEntryByOrdinal.put(entry.ordinal(), entry);
		}
	}

	@Test
	public void testSlotSize() {
		FlexRay flexRay = createBusStructure();

		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.MAX_DATA));
		long expectedNs = (int) Math.ceil((262 * 8 * 1000000) / ((double) 20));
		assertEquals(expectedNs, flexRay.getSlotSize().toNanos());

		flexRay.setMode(new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY));
		expectedNs = (int) Math.ceil((262 * 8 * 1000000) / ((double) 10));
		System.out.println(expectedNs);
		assertEquals(expectedNs, flexRay.getSlotSize().toNanos());
	}

	@Test
	public void testCycleTime() {
		FlexRay flexRay = createBusStructure();

		long expectedNs = flexRay.getSlotSize().toNanos()
				* (flexRay.connectedComponents.size() + FlexRay.DYNAMIC_SLOTS);
		assertEquals(expectedNs, flexRay.getCycleTime().toNanos());
	}

	@Test
	public void testMode() {
		FlexRay flexray = createBusStructure();

		// Mode setup
		FlexRayOperationMode modeOne = new FlexRayOperationMode(FlexRayOperationModeEnum.REDUNDANCY);
		FlexRayOperationMode modeTwo = new FlexRayOperationMode(FlexRayOperationModeEnum.MAX_DATA);

		// Test of the set and get mode functions
		flexray.setMode(modeOne);
		assertEquals(modeOne, flexray.getMode());

		flexray.setMode(modeTwo);
		assertEquals(modeTwo, flexray.getMode());
	}

	@Test(expected = IllegalArgumentException.class)
	public void testRegisterMessageFromInvalidController() {
		Bus bus = createBusStructure();

		Optional<EEComponent> comp = BusUtils.findComponentWithID(bus.connectedComponents, "TestComponent 1");

		assertNotNull(comp);
		String msg = "FromIllegalController";
		BusMessage illegalControllerMsg = new BusMessage(msg, msg.length(),
				NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, MessageType.SEND, comp.get());
		bus.registerMessage(illegalControllerMsg);
	}

	@Test(expected = IllegalArgumentException.class)
	public void testRegisterMessageToInvalidController() {
		Bus bus = createBusStructure();
		
		Optional<EEComponent> comp = BusUtils.findComponentWithID(bus.connectedComponents, "TestComponent 1");
		assertTrue(comp.isPresent());
		EEComponent illegalController = new TestComponent(EEsim, "IllegalID");

		String msg = "ToIllegalController";
		BusMessage illegalControllerMsg = new BusMessage(msg, msg.length(),
				NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, MessageType.SEND, illegalController);
		illegalControllerMsg.setControllerID(comp.get().getID());
		bus.registerMessage(illegalControllerMsg);
	}

	public void testRegisterMessage() {
		FlexRay mainBus = createBusStructure();
		Bus subBus1 = BusUtils.findConnectedBuses(mainBus.connectedComponents).iterator().next();
		Bus subBus2 = BusUtils.findConnectedBuses(subBus1.connectedComponents).iterator().next();

		Optional<EEComponent> mainComp1 = BusUtils.findComponentWithID(mainBus.connectedComponents, "TestComponent 1");
		assertTrue(mainComp1.isPresent());
		Optional<EEComponent> mainComp2 = BusUtils.findComponentWithID(mainBus.connectedComponents, "TestComponent 2");
		assertTrue(mainComp2.isPresent());
		Optional<EEComponent> sub1Comp = BusUtils.findComponentWithID(subBus1.connectedComponents, "TestComponent 5");
		assertTrue(sub1Comp.isPresent());
		Optional<EEComponent> sub2Comp = BusUtils.findComponentWithID(subBus2.connectedComponents, "TestComponent 8");
		assertTrue(sub2Comp.isPresent());

		String msg = "Local";
		BusMessage localMsg = new BusMessage(msg, msg.length(), NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE,
				Instant.EPOCH, MessageType.SEND, mainComp2.get());
		localMsg.setControllerID(mainComp1.get().getID());
		mainBus.registerMessage(localMsg);

		msg = "1hop";
		BusMessage oneHopMsg = new BusMessage(msg, msg.length(), NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE,
				Instant.EPOCH, MessageType.SEND, sub1Comp.get());
		oneHopMsg.setControllerID(mainComp1.get().getID());
		mainBus.registerMessage(oneHopMsg);

		msg = "2hop";
		BusMessage twoHopMsg = new BusMessage(msg, msg.length(), NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE,
				Instant.EPOCH, MessageType.SEND, sub2Comp.get());
		twoHopMsg.setControllerID(mainComp2.get().getID());
		mainBus.registerMessage(twoHopMsg);

		Map<String, PriorityQueue<BusMessage>> messagesByControllerId = mainBus.getMessagesByControllerId();
		assertEquals(2, messagesByControllerId.get(mainComp1.get().getID()).size());
		assertEquals(1, messagesByControllerId.get(mainComp2.get().getID()).size());
	}

	@Test
	public void testGetNextDynamicMessage() {
		FlexRay flexray = createBusStructure();

		BusMessage empty = flexray.getNextDynamicMessage();
		assertTrue(empty == null);

		BusMessage c01 = createNregisterMessage(flexray, "c01", "TestComponent 0", "TestComponent 1", 1, 0);
		BusMessage msg = flexray.getNextDynamicMessage();
		assertTrue(msg != null);
		assertEquals(msg, c01);

		BusMessage c02 = createNregisterMessage(flexray, "c02", "TestComponent 0", "TestComponent 2", 254, 1);
		msg = flexray.getNextDynamicMessage();
		assertTrue(msg != null);
		assertEquals(msg, c02);

		BusMessage c11 = createNregisterMessage(flexray, "c11", "TestComponent 1", "TestComponent 2", 127, 2);
		msg = flexray.getNextDynamicMessage();
		assertTrue(msg != null);
		assertEquals(msg, c11);

		BusMessage c12 = createNregisterMessage(flexray, "c12", "TestComponent 1", "TestComponent 3", 127, 3);
		msg = flexray.getNextDynamicMessage();
		assertTrue(msg != null);
		assertEquals(msg, c12);

		createNregisterMessage(flexray, "", "TestComponent 1", "TestComponent 4", 1, 0);
		msg = flexray.getNextDynamicMessage();
		assertTrue(msg != null);
		assertEquals(msg, c12);
	}

	@Test
	public void testFillStaticSegment() {
		FlexRay flexray = createBusStructure();
		createNregisterMessages(flexray);
		Map<Integer, List<BusMessage>> msgsByEndCycle = new HashMap<Integer, List<BusMessage>>();
		for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
			BusMessage msgArray[] = controllerMsgs.toArray(new BusMessage[controllerMsgs.size()]);
			Arrays.parallelSort(msgArray, controllerMsgs.comparator());
			int totalBytes = 0;
			for (int i = 0; i < controllerMsgs.size(); i++) {
				BusMessage cur = msgArray[i];
				totalBytes += cur.getMessageLen();
				int cycle = totalBytes / FlexRay.MAX_SLOT_PAYLOAD;
				if ((totalBytes % FlexRay.MAX_SLOT_PAYLOAD) != 0) {
					cycle++;
				}
				System.out.println("Total bytes: " + totalBytes + " Cycle: " + cycle);
				List<BusMessage> finishedMsgs = msgsByEndCycle.getOrDefault(cycle, new ArrayList<BusMessage>());
				finishedMsgs.add(cur);
				msgsByEndCycle.put(cycle, finishedMsgs);
			}
		}
		int curCycle = 0;

		Map<String, Integer> transmittedBytesByControllerId = new HashMap<String, Integer>();
		Map<String, Boolean> firstByControllerId = new HashMap<String, Boolean>();
		for (String controllerID : flexray.getMessagesByControllerId().keySet()) {
			transmittedBytesByControllerId.put(controllerID, 0);
			firstByControllerId.put(controllerID, true);
		}

		for (Map.Entry<Integer, List<BusMessage>> entry : msgsByEndCycle.entrySet()) {
			System.out.println("-----------------------");
			for (Map.Entry<String, Boolean> first : firstByControllerId.entrySet()) {
				first.setValue(true);
			}

			for (BusMessage msg : entry.getValue()) {
				assertTrue(!msg.isTransmitted());
				if (firstByControllerId.get(msg.getControllerID())) {
					int transmittedBytes = transmittedBytesByControllerId.get(msg.getControllerID());
					System.out.println(msg.getMessage().toString() + "Expected: " + transmittedBytes + "Actual: "
							+ msg.getTransmittedBytes());
					assertEquals(transmittedBytes, msg.getTransmittedBytes());
					firstByControllerId.put(msg.getControllerID(), false);
				} else {
					System.out.println(
							msg.getMessage().toString() + "Expected: " + 0 + "Actual: " + msg.getTransmittedBytes());
					assertEquals(0, msg.getTransmittedBytes());
				}

			}
			for (; curCycle < entry.getKey(); curCycle++) {
				flexray.fillStaticSegment(Instant.EPOCH);
				for (Map.Entry<String, Integer> transmittedBytesEntry : transmittedBytesByControllerId.entrySet()) {
					int transmittedBytes = transmittedBytesEntry.getValue();
					transmittedBytesEntry.setValue(transmittedBytes + FlexRay.MAX_SLOT_PAYLOAD);
				}
			}
			for (BusMessage msg : entry.getValue()) {
				System.out.println(msg.getMessage().toString());
				assertTrue(msg.isTransmitted());
				int transmittedBytes = transmittedBytesByControllerId.get(msg.getControllerID());
				transmittedBytesByControllerId.put(msg.getControllerID(), transmittedBytes - msg.getMessageLen());
			}
		}
	}

	@Test
	public void testFillDynamicSegment() {

		FlexRay flexray = createBusStructure();
		List<BusMessage> msgs = createNregisterMessages(flexray);

		Map<Integer, List<BusMessage>> msgsByEndCycle = new HashMap<Integer, List<BusMessage>>();
		BusMessage msgArray[] = msgs.toArray(new BusMessage[msgs.size()]);
		Arrays.parallelSort(msgArray, new BusMessageComparatorIdDesc());
		int totalBytes = 0;
		int bytesPerCycle = FlexRay.DYNAMIC_SLOTS * FlexRay.MAX_SLOT_PAYLOAD;
		for (int i = 0; i < msgArray.length; i++) {
			BusMessage cur = msgArray[i];
			totalBytes += cur.getMessageLen();
			int cycle = totalBytes / bytesPerCycle;
			if ((totalBytes % bytesPerCycle) != 0) {
				cycle++;
			}
			System.out.println("Total bytes: " + totalBytes + " Cycle: " + cycle);
			List<BusMessage> finishedMsgs = msgsByEndCycle.getOrDefault(cycle, new ArrayList<BusMessage>());
			finishedMsgs.add(cur);
			msgsByEndCycle.put(cycle, finishedMsgs);
		}

		int curCycle = 0;
		boolean first = true;
		int transmittedBytes = 0;

		for (Map.Entry<Integer, List<BusMessage>> entry : msgsByEndCycle.entrySet()) {
			System.out.println("-----------------------");
			first = true;

			for (BusMessage msg : entry.getValue()) {
				assertTrue(!msg.isTransmitted());
				if (first) {
					System.out.println(msg.getMessage().toString() + " Expected: " + transmittedBytes
							+ " transmittedBytes. Actual: " + msg.getTransmittedBytes());
					assertEquals(transmittedBytes, msg.getTransmittedBytes());
					first = false;
				} else {
					System.out.println(msg.getMessage().toString() + "Expected: " + 0 + "transmittedBytes. Actual: "
							+ msg.getTransmittedBytes());
					assertEquals(0, msg.getTransmittedBytes());
				}

			}

			for (; curCycle < entry.getKey(); curCycle++) {
				flexray.fillDynamicSegment(Instant.EPOCH);
				transmittedBytes += bytesPerCycle;
			}

			for (BusMessage msg : entry.getValue()) {
				System.out.println("Expected to finish: " + msg.getMessage().toString());
				assertTrue(msg.isTransmitted());
				transmittedBytes -= msg.getMessageLen();
			}
		}
	}

	@Test
	public void testGetNextFinishTime() {
		FlexRay flexray = createBusStructure();
		createNregisterMessages(flexray);
		boolean nextTransmitted = false;

		PriorityQueue<BusMessage> firstMsgs = new PriorityQueue<BusMessage>(new BusMessageComparatorIdDesc());
		for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
			if (!controllerMsgs.isEmpty()) {
				firstMsgs.add(new BusMessage(controllerMsgs.peek()));
			}
		}

		int cycles = 0;
		while (!nextTransmitted) {
			cycles++;
			// static segment
			for (BusMessage msg : firstMsgs) {
				int actual = msg.transmitBytes(254, 0);
				if (actual < 254) {
					nextTransmitted = true;
				}
			}
			// dynamic segment
			int actual = firstMsgs.peek().transmitBytes((254 * FlexRay.DYNAMIC_SLOTS), 0);
			if (actual < (254 * FlexRay.DYNAMIC_SLOTS)) {
				nextTransmitted = true;
			}
		}
		Instant expected = flexray.currentTime.plusNanos((flexray.getCycleTime().toNanos() * cycles));
		assertEquals(expected, flexray.getNextFinishTime());
	}

	@Test
	public void testsimulateForRandomized() {
		FlexRay flexray = createBusStructure();
		createNregisterMessages(flexray);

		PriorityQueue<BusMessage> firstMsgs = new PriorityQueue<BusMessage>(new BusMessageComparatorIdDesc());
		for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
			if (!controllerMsgs.isEmpty()) {
				firstMsgs.add(controllerMsgs.peek());
			}
		}

		while (!firstMsgs.isEmpty()) {
			BusMessage cur = firstMsgs.poll();
			// transmitted during static and dynamic segments
			int msgCycles = (int) Math
					.ceil(cur.getRemainingBytes() / ((double) (FlexRay.MAX_SLOT_PAYLOAD * (FlexRay.DYNAMIC_SLOTS + 1))));
			flexray.simulateFor(flexray.getCycleTime().multipliedBy(msgCycles));
			assertTrue(cur.isTransmitted());

			firstMsgs.clear();
			for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
				if (!controllerMsgs.isEmpty()) {
					firstMsgs.add(controllerMsgs.peek());
				}
			}
		}
	}

	@Test
	public void testsimulateFor() {
		FlexRay flexray = createBusStructure();

		//finish in third cycle (static)
		BusMessage c01 = createNregisterMessage(flexray, "c01", "TestComponent 0", "TestComponent 1", 127, 0);
		//finish in second cycle  (dynamic 1.5 slots)
		BusMessage c02 = createNregisterMessage(flexray, "c02", "TestComponent 0", "TestComponent 1",
				(254 + 254 + 254 + 381), 4);
		
		//finish in third cycle (static)
		BusMessage c11 = createNregisterMessage(flexray, "c11", "TestComponent 1", "TestComponent 1", 100, 0);
		//finish in second cycle  (dynamic 4 slots)
		BusMessage c12 = createNregisterMessage(flexray, "c12", "TestComponent 1", "TestComponent 1", (254 + 254 + 635),
				3);

		//finish in first cycle (static)
		BusMessage c21 = createNregisterMessage(flexray, "c21", "TestComponent 2", "TestComponent 1", 100, 0);
		//finish in first cycle (static)
		BusMessage c22 = createNregisterMessage(flexray, "c22", "TestComponent 2", "TestComponent 1", 100, 0);
		//finish in first cycle (static)
		BusMessage c23 = createNregisterMessage(flexray, "c23", "TestComponent 2", "TestComponent 1", 54, 0);

		//finish in third cycle (static)
		BusMessage c31 = createNregisterMessage(flexray, "c31", "TestComponent 3", "TestComponent 1", 100, 0);
		//finish in second cycle (static)
		BusMessage c32 = createNregisterMessage(flexray, "c32", "TestComponent 3", "TestComponent 1", 200, 1);
		//finish in first cycle (dynamic after 3 slots) 
		BusMessage c33 = createNregisterMessage(flexray, "c33", "TestComponent 3", "TestComponent 1", (154 + 762), 5);
		//finish in first cycle (static)
		BusMessage c34 = createNregisterMessage(flexray, "c34", "TestComponent 3", "TestComponent 1", 100, 6);
		
		long slotSizeNs = Math.toIntExact(flexray.getSlotSize().toNanos());
		long totalStaticSegmentSizeNs = Math.toIntExact(flexray.getStaticSegmentSize().toNanos());
		long cycleTimeNs = Math.toIntExact(flexray.getCycleTime().toNanos());
		
		Instant currentTime = Instant.EPOCH;
		flexray.simulateFor(Duration.ofNanos(cycleTimeNs/2));
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(2).toNanos());
		assertEquals(currentTime, flexray.currentTime);
		flexray.simulateFor(flexray.getCycleTime().dividedBy(3));
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		assertEquals(currentTime, flexray.currentTime);
		flexray.simulateFor(flexray.getCycleTime().dividedBy(3));
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		assertEquals(currentTime, flexray.currentTime);
		flexray.simulateFor(flexray.getCycleTime().dividedBy(2));
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(2).toNanos());
		assertEquals(currentTime, flexray.currentTime);
		flexray.simulateFor(flexray.getCycleTime().dividedBy(3));
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		assertEquals(currentTime, flexray.currentTime);
		flexray.simulateFor(flexray.getCycleTime());

		assertTrue(0 < Duration.between(Instant.EPOCH, c34.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c34.getFinishTime()).toNanos());
		
		System.out.println("Expected: " + (totalStaticSegmentSizeNs + (slotSizeNs * 3)) + "; Actual: " + Duration.between(Instant.EPOCH, c33.getFinishTime()).toNanos());
		assertEquals((totalStaticSegmentSizeNs + (slotSizeNs * 3)), Duration.between(Instant.EPOCH, c33.getFinishTime()).toNanos());
		
		assertTrue(0 < Duration.between(Instant.EPOCH, c21.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c21.getFinishTime()).toNanos());
		
		assertTrue(0 < Duration.between(Instant.EPOCH, c22.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c22.getFinishTime()).toNanos());
		
		assertTrue(0 < Duration.between(Instant.EPOCH, c23.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c23.getFinishTime()).toNanos());
		
		System.out.println("Expected: " + (cycleTimeNs + totalStaticSegmentSizeNs + slotSizeNs) + "; Actual: " + Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs + slotSizeNs) < Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		System.out.println("Expected: " + (cycleTimeNs + totalStaticSegmentSizeNs + (2* slotSizeNs)) + "; Actual: " + Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs + (2* slotSizeNs)) > Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		
		System.out.println(cycleTimeNs + totalStaticSegmentSizeNs);
		System.out.println("Expected: " + (cycleTimeNs * 2) + "; Actual: " + Duration.between(Instant.EPOCH, c12.getFinishTime()).toNanos());
		assertEquals((cycleTimeNs * 2), Duration.between(Instant.EPOCH, c12.getFinishTime()).toNanos());
		
		assertTrue(cycleTimeNs < Duration.between(Instant.EPOCH, c32.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c32.getFinishTime()).toNanos());
		
		System.out.println("Expected: " + (cycleTimeNs * 2) + "; Actual: " + Duration.between(Instant.EPOCH, c01.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c01.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c01.getFinishTime()).toNanos());
		
		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c11.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c11.getFinishTime()).toNanos());
		
		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c31.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c31.getFinishTime()).toNanos());
	}

	private FlexRay createBusStructure() {
		List<EEComponent> mainComponents = new ArrayList<EEComponent>();
		List<EEComponent> subComponents1 = new ArrayList<EEComponent>();
		List<EEComponent> subComponents2 = new ArrayList<EEComponent>();
		int i = 0;
		for (; i < 5; i++) {
			mainComponents.add(new TestComponent(EEsim, String.valueOf(i)));
		}
		for (; i < 8; i++) {
			subComponents1.add(new TestComponent(EEsim, String.valueOf(i)));
		}
		for (; i < 15; i++) {
			subComponents2.add(new TestComponent(EEsim, String.valueOf(i)));
		}

		List<BusEntry> messages = new ArrayList<BusEntry>();
		messages.add(BusEntry.CONSTANT_WHEELBASE);

		FlexRay sub2 = new FlexRay(EEsim);//subComponents2
		for(EEComponent component: subComponents2){
			sub2.registerComponent(component, messages);
		}

		subComponents1.add(sub2);
		FlexRay sub1 = new FlexRay(EEsim);//subComponents1
		for(EEComponent component: subComponents1){
			sub1.registerComponent(component, messages);
		}

		mainComponents.add(sub1);
		FlexRay main = new FlexRay(EEsim);//mainComponents
		for(EEComponent component: mainComponents){
			main.registerComponent(component, messages);
		}

		return main;
	}

	private BusMessage createNregisterMessage(FlexRay flexray, Object message, String senderID, String receiverID,
			int messageLength, int priority) {
		Optional<EEComponent> sender = BusUtils.findComponentWithID(flexray.connectedComponents, senderID);
		assertTrue(sender.isPresent());
		Optional<EEComponent> receiver = BusUtils.findComponentWithID(flexray.connectedComponents, receiverID);
		assertTrue(receiver.isPresent());

		assertTrue(busEntryByOrdinal.size() > priority);

		BusMessage msg = new BusMessage(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
				MessageType.SEND, receiver.get());
		msg.setControllerID(sender.get().getID());
		flexray.registerMessage(msg);

		return msg;
	}

	private List<BusMessage> createNregisterMessages(FlexRay flexray) {
		List<BusMessage> msgs = new ArrayList<BusMessage>();

		List<String> controllerIds = new ArrayList<String>();
		for (String controllerId : flexray.getMessagesByControllerId().keySet()) {
			if (!controllerId.startsWith("Bus")) {
				controllerIds.add(controllerId);
			}

		}

		// make sure ordering is deterministic => no two messages with same priority
		List<Integer> priorities = new ArrayList<Integer>();
		for (int j = 0; j < busEntryByOrdinal.size(); j++) {
			priorities.add(j);
		}

		Random rand = new Random();
		for (int j = 0; j < busEntryByOrdinal.size(); j++) {
			int senderID = rand.nextInt(controllerIds.size());
			int receiverID = rand.nextInt(controllerIds.size());
			int messageLength = rand.nextInt(1500);
			int priority = priorities.remove(rand.nextInt(priorities.size()));
			msgs.add(createNregisterMessage(flexray, j, controllerIds.get(senderID), controllerIds.get(receiverID),
					messageLength, priority));
		}
		return msgs;
	}
	
	private int getTransmitTime(int bytes, int dataRate) {
		return (int) Math.ceil((bytes* 1000000 * 8) / ((double) dataRate));
	}
}
