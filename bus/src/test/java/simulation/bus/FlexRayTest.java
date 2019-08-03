package simulation.bus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.UUID;

import static commons.controller.commons.BusEntry.*;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.BeforeClass;
import org.junit.Test;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.TestComponent;

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
		System.out.println(flexRay.getSlotSize().toMillis());
		assertEquals(expectedNs, flexRay.getSlotSize().toNanos());
	}

	@Test
	public void testCycleTime() {
		FlexRay flexRay = createBusStructure();

		long expectedNs = flexRay.getSlotSize().toNanos()
				* (flexRay.getConnectedComponents().size() + FlexRay.DYNAMIC_SLOTS);
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

		EEComponent comp = bus.getConnectedComponents().get(0);
		UUID invalidID = UUID.randomUUID();
		String msg = "FromIllegalController";
		BusMessage illegalControllerMsg = new BusMessage(msg, msg.length(),
				NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, invalidID, comp);
		bus.registerMessage(illegalControllerMsg);
	}

	@Test(expected = IllegalArgumentException.class)
	public void testRegisterMessageToInvalidController() {
		Bus bus = createBusStructure();

		EEComponent comp = bus.getConnectedComponents().get(0);
		EEComponent illegalController = new TestComponent(EEsim);

		String msg = "ToIllegalController";
		BusMessage illegalControllerMsg = new BusMessage(msg, msg.length(),
				NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, comp.getId(), illegalController);
		bus.registerMessage(illegalControllerMsg);
	}

	@Test
	public void testGetNextDynamicMessage() throws NoSuchMethodException, SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {
		FlexRay flexray = createBusStructure();

		Method getNextDymaicMessage = FlexRay.class.getDeclaredMethod("getNextDynamicMessage");
		getNextDymaicMessage.setAccessible(true);
		BusMessage empty = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(empty == null);

		BusMessage c01 = createNregisterMessage(flexray, "c01", 0, 1, 0);
		BusMessage msg = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(msg != null);
		assertEquals(msg, c01);

		BusMessage c02 = createNregisterMessage(flexray, "c02", 0, 254, 1);
		msg = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(msg != null);
		assertEquals(msg, c02);

		BusMessage c11 = createNregisterMessage(flexray, "c11", 1, 127, 2);
		msg = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(msg != null);
		assertEquals(msg, c11);

		BusMessage c12 = createNregisterMessage(flexray, "c12", 1, 127, 3);
		msg = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(msg != null);
		assertEquals(msg, c12);

		createNregisterMessage(flexray, "", 1, 1, 0);
		msg = (BusMessage) getNextDymaicMessage.invoke(flexray);
		assertTrue(msg != null);
		assertEquals(msg, c12);
	}

	@Test
	public void testFillStaticSegment() throws NoSuchMethodException, SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {
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

		Map<UUID, Integer> transmittedBytesByControllerId = new HashMap<UUID, Integer>();
		Map<UUID, Boolean> firstByControllerId = new HashMap<UUID, Boolean>();
		for (UUID controllerID : flexray.getMessagesByControllerId().keySet()) {
			transmittedBytesByControllerId.put(controllerID, 0);
			firstByControllerId.put(controllerID, true);
		}

		for (Map.Entry<Integer, List<BusMessage>> entry : msgsByEndCycle.entrySet()) {
			System.out.println("-----------------------");
			for (Map.Entry<UUID, Boolean> first : firstByControllerId.entrySet()) {
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

			Method fillStaticSegment = FlexRay.class.getDeclaredMethod("fillStaticSegment", Instant.class);
			fillStaticSegment.setAccessible(true);
			for (; curCycle < entry.getKey(); curCycle++) {
				fillStaticSegment.invoke(flexray, Instant.EPOCH);
				for (Map.Entry<UUID, Integer> transmittedBytesEntry : transmittedBytesByControllerId.entrySet()) {
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
	public void testFillDynamicSegment() throws NoSuchMethodException, SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {

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

			for (Method mth : FlexRay.class.getMethods()) {
				System.out.println(mth.getName());
			}
			Method fillDynamicSegment = FlexRay.class.getDeclaredMethod("fillDynamicSegment", Instant.class);
			fillDynamicSegment.setAccessible(true);
			for (; curCycle < entry.getKey(); curCycle++) {
				fillDynamicSegment.invoke(flexray, Instant.EPOCH);
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
		
		BusMessage dynamic = createNregisterMessage(flexray, "finished in last dynamic slot", 0, (4 * 254 + 250), 0);
		BusMessage copy = new BusMessage(dynamic);
		Instant finishTime = flexray.getNextFinishTime();
		assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
		Instant minFinishTime = Instant.EPOCH.plus(flexray.getStaticSegmentSize())
				.plus(flexray.getSlotSize().multipliedBy(3));
		assertTrue(minFinishTime.isBefore(finishTime));
		assertTrue(minFinishTime.plus(flexray.getSlotSize()).isAfter(finishTime));
		flexray.simulateUntil(finishTime);
		assertTrue(dynamic.isTransmitted());
		assertEquals(finishTime, dynamic.getFinishTime());

		dynamic = createNregisterMessage(flexray, "finished at end of last dynamic slot", 0, 4, 0);
		copy = new BusMessage(dynamic);
		finishTime = flexray.getNextFinishTime();
		assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
		assertEquals(Instant.EPOCH.plus(flexray.getCycleTime()), finishTime);
		flexray.simulateUntil(finishTime);
		assertTrue(dynamic.isTransmitted());
		assertEquals(finishTime, dynamic.getFinishTime());

		//start from incomplete dynamic slot
		dynamic = createNregisterMessage(flexray, "finished in last dynamic slot", 0, (4 * 254 + 123), 0);
		copy = new BusMessage(dynamic);
		finishTime = flexray.getNextFinishTime();
		assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
		minFinishTime = Instant.EPOCH.plus(flexray.getCycleTime()).plus(flexray.getStaticSegmentSize())
				.plus(flexray.getSlotSize().multipliedBy(3));
		assertTrue(minFinishTime.isBefore(finishTime));
		assertTrue(minFinishTime.plus(flexray.getSlotSize()).isAfter(finishTime));
		flexray.simulateUntil(finishTime);
		assertTrue(dynamic.isTransmitted());
		assertEquals(finishTime, dynamic.getFinishTime());
		
		// create message that takes up dynamic slots
		dynamic = createNregisterMessage(flexray, "dynamic", 0, Integer.MAX_VALUE, 20);

		Random rand = new Random();
		Instant lastFinishedCycle = Instant.EPOCH.plus(flexray.getCycleTime().multipliedBy(2));
		for (int i = 2; i < flexray.getConnectedComponents().size(); i++) {
			System.out.println("--------------new Message-----------------");
			int slotNumber = 0;
			int messageLen = rand.nextInt(253) + 1;
			
			//start from incomplete static segment
			BusMessage controller = createNregisterMessage(flexray, "static", i-1, messageLen, 0);
			for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
				if (!controllerMsgs.isEmpty() && controllerMsgs.peek() != dynamic) {
					break;
				}
				slotNumber++;
			}
			copy = new BusMessage(controller);
			finishTime = flexray.getNextFinishTime();
			assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());
			if(lastFinishedCycle.plus(flexray.getCycleTime()).isBefore(finishTime)) {
				lastFinishedCycle= lastFinishedCycle.plus(flexray.getCycleTime());
			}
			minFinishTime = lastFinishedCycle.plus(flexray.getSlotSize().multipliedBy(slotNumber));
			assertTrue(minFinishTime.isBefore(finishTime));
			assertTrue(minFinishTime.plus(flexray.getSlotSize()).isAfter(finishTime));
			flexray.simulateUntil(finishTime);
			assertTrue(controller.isTransmitted());
			assertEquals(finishTime, controller.getFinishTime());
			
			slotNumber = 0;
			controller = createNregisterMessage(flexray, "static", i, messageLen, 0);
			for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
				if (!controllerMsgs.isEmpty() && controllerMsgs.peek() != dynamic) {
					break;
				}
				slotNumber++;
			}
			copy = new BusMessage(controller);
			finishTime = flexray.getNextFinishTime();
			assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());
			if(lastFinishedCycle.plus(flexray.getCycleTime()).isBefore(finishTime)) {
				lastFinishedCycle= lastFinishedCycle.plus(flexray.getCycleTime());
			}
			minFinishTime = lastFinishedCycle.plus(flexray.getSlotSize().multipliedBy(slotNumber));
			assertTrue(minFinishTime.isBefore(finishTime));
			assertTrue(minFinishTime.plus(flexray.getSlotSize()).isAfter(finishTime));
			flexray.simulateUntil(finishTime);
			assertTrue(controller.isTransmitted());
			assertEquals(finishTime, controller.getFinishTime());

			controller = createNregisterMessage(flexray, "static", i, 254 - messageLen, 0);
			copy = new BusMessage(controller);
			finishTime = flexray.getNextFinishTime();
			assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());			assertEquals(minFinishTime.plus(flexray.getSlotSize()), finishTime);
			flexray.simulateUntil(finishTime);
			assertTrue(controller.isTransmitted());
			assertEquals(finishTime, controller.getFinishTime());
		}

	}

	@Test
	public void testGetNextFinishTimeRandomized() {
		FlexRay flexray = createBusStructure();
		List<BusMessage> msgs = createNregisterMessages(flexray);
		int i = 0;
		for (;i < 2 && !msgs.isEmpty(); i++) {
			List<BusMessage> firstMsgs = new ArrayList<BusMessage>();
			for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
				PriorityQueue<BusMessage> copy = new PriorityQueue<BusMessage>(controllerMsgs); 
				BusMessage cur = copy.poll();
				while(cur != null && cur.isTransmitted()) {
					cur = copy.poll();
				}
				if(cur != null) {
					firstMsgs.add(cur);
				}
			}
			System.out.println("--------------New Message-------------");
			System.out.println("--------------"+ msgs.size() + " remaining msgs-------------");
			Instant nextFinishTime = flexray.getNextFinishTime();
			flexray.simulateUntil(nextFinishTime);

			boolean transmitted = false;
			for(BusMessage msg : firstMsgs) {
				if (msg.isTransmitted()) {
					System.out.println(msg);
					assertEquals(msg.getFinishTime(), nextFinishTime);
					transmitted = true;
					msgs.remove(msg);
					i = 0;
				}
			}
			assertTrue(transmitted);
		}

		assertEquals(0, msgs.size());
	}

	@Test
	public void testsimulateUntilRandomized() throws NoSuchMethodException, SecurityException, IllegalAccessException,
			IllegalArgumentException, InvocationTargetException {
		FlexRay flexray = createBusStructure();
		createNregisterMessages(flexray);

		PriorityQueue<BusMessage> firstMsgs = new PriorityQueue<BusMessage>(new BusMessageComparatorIdDesc());
		for (PriorityQueue<BusMessage> controllerMsgs : flexray.getMessagesByControllerId().values()) {
			if (!controllerMsgs.isEmpty()) {
				firstMsgs.add(controllerMsgs.peek());
			}
		}
		Method simulateUntil = FlexRay.class.getDeclaredMethod("simulateUntil", Instant.class);
		simulateUntil.setAccessible(true);
		while (!firstMsgs.isEmpty()) {
			BusMessage cur = firstMsgs.poll();
			// transmitted during static and dynamic segments
			int msgCycles = (int) Math.ceil(
					cur.getRemainingBytes() / ((double) (FlexRay.MAX_SLOT_PAYLOAD * (FlexRay.DYNAMIC_SLOTS + 1))));
			simulateUntil.invoke(flexray,
					flexray.getCurrentTime().plus(flexray.getCycleTime().multipliedBy(msgCycles)));
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
	public void testsimulateUntil() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException,
			NoSuchMethodException, SecurityException {
		FlexRay flexray = createBusStructure();

		// finish in third cycle (static)
		BusMessage c01 = createNregisterMessage(flexray, "c01", 0, 127, 0);
		// finish in second cycle (dynamic 1.5 slots)
		BusMessage c02 = createNregisterMessage(flexray, "c02", 0, (254 + 254 + 254 + 381), 4);

		// finish in third cycle (static)
		BusMessage c11 = createNregisterMessage(flexray, "c11", 1, 100, 0);
		// finish in second cycle (dynamic 4 slots)
		BusMessage c12 = createNregisterMessage(flexray, "c12", 1, (254 + 254 + 635), 3);

		// finish in first cycle (static)
		BusMessage c21 = createNregisterMessage(flexray, "c21", 2, 100, 0);
		// finish in first cycle (static)
		BusMessage c22 = createNregisterMessage(flexray, "c22", 2, 100, 0);
		// finish in first cycle (static)
		BusMessage c23 = createNregisterMessage(flexray, "c23", 2, 54, 0);

		// finish in third cycle (static)
		BusMessage c31 = createNregisterMessage(flexray, "c31", 3, 100, 0);
		// finish in second cycle (static)
		BusMessage c32 = createNregisterMessage(flexray, "c32", 3, 200, 1);
		// finish in first cycle (dynamic after 3 slots)
		BusMessage c33 = createNregisterMessage(flexray, "c33", 3, (154 + 762), 5);
		// finish in first cycle (static)
		BusMessage c34 = createNregisterMessage(flexray, "c34", 3, 100, 6);

		long slotSizeNs = flexray.getSlotSize().toNanos();
		long totalStaticSegmentSizeNs = Math.toIntExact(flexray.getStaticSegmentSize().toNanos());
		long cycleTimeNs = flexray.getCycleTime().toNanos();

		Method simulateUntil = FlexRay.class.getDeclaredMethod("simulateUntil", Instant.class);
		simulateUntil.setAccessible(true);

		Instant currentTime = Instant.EPOCH;
		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(2).toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(2).toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		currentTime = currentTime.plusNanos(flexray.getCycleTime().dividedBy(3).toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		currentTime = currentTime.plusNanos(flexray.getCycleTime().toNanos());
		simulateUntil.invoke(flexray, currentTime);
		assertEquals(currentTime, flexray.getCurrentTime());

		assertTrue(0 < Duration.between(Instant.EPOCH, c34.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c34.getFinishTime()).toNanos());

		System.out.println("Expected: " + (totalStaticSegmentSizeNs + (slotSizeNs * 3)) + "; Actual: "
				+ Duration.between(Instant.EPOCH, c33.getFinishTime()).toNanos());
		assertEquals((totalStaticSegmentSizeNs + (slotSizeNs * 3)),
				Duration.between(Instant.EPOCH, c33.getFinishTime()).toNanos());

		assertTrue(0 < Duration.between(Instant.EPOCH, c21.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c21.getFinishTime()).toNanos());

		assertTrue(0 < Duration.between(Instant.EPOCH, c22.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c22.getFinishTime()).toNanos());

		assertTrue(0 < Duration.between(Instant.EPOCH, c23.getFinishTime()).toNanos());
		assertTrue(totalStaticSegmentSizeNs > Duration.between(Instant.EPOCH, c23.getFinishTime()).toNanos());

		System.out.println("Expected: " + (cycleTimeNs + totalStaticSegmentSizeNs + slotSizeNs) + "; Actual: "
				+ Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs + slotSizeNs) < Duration
				.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		System.out.println("Expected: " + (cycleTimeNs + totalStaticSegmentSizeNs + (2 * slotSizeNs)) + "; Actual: "
				+ Duration.between(Instant.EPOCH, c02.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs + (2 * slotSizeNs)) > Duration
				.between(Instant.EPOCH, c02.getFinishTime()).toNanos());

		System.out.println(cycleTimeNs + totalStaticSegmentSizeNs);
		System.out.println("Expected: " + (cycleTimeNs * 2) + "; Actual: "
				+ Duration.between(Instant.EPOCH, c12.getFinishTime()).toNanos());
		assertEquals((cycleTimeNs * 2), Duration.between(Instant.EPOCH, c12.getFinishTime()).toNanos());

		assertTrue(cycleTimeNs < Duration.between(Instant.EPOCH, c32.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c32.getFinishTime())
				.toNanos());

		System.out.println("Expected: " + (cycleTimeNs * 2) + "; Actual: "
				+ Duration.between(Instant.EPOCH, c01.getFinishTime()).toNanos());
		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c01.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c01.getFinishTime())
				.toNanos());

		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c11.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c11.getFinishTime())
				.toNanos());

		assertTrue((cycleTimeNs * 2) < Duration.between(Instant.EPOCH, c31.getFinishTime()).toNanos());
		assertTrue(((cycleTimeNs * 2) + totalStaticSegmentSizeNs) > Duration.between(Instant.EPOCH, c31.getFinishTime())
				.toNanos());
	}

	private FlexRay createBusStructure() {
		List<EEComponent> mainComponents = new ArrayList<EEComponent>();
		List<EEComponent> subComponents1 = new ArrayList<EEComponent>();
		List<EEComponent> subComponents2 = new ArrayList<EEComponent>();
		int i = 0;
		for (; i < 5; i++) {
			mainComponents.add(new TestComponent(EEsim));
		}
		for (; i < 8; i++) {
			subComponents1.add(new TestComponent(EEsim));
		}
		for (; i < 15; i++) {
			subComponents2.add(new TestComponent(EEsim));
		}

		FlexRay sub2 = new FlexRay(EEsim);// subComponents2
		for (EEComponent component : subComponents2) {
			sub2.registerComponent(component);
		}

		FlexRay sub1 = new FlexRay(EEsim);// subComponents1
		for (EEComponent component : subComponents1) {
			sub1.registerComponent(component);
		}

		new Bridge(EEsim, new ImmutablePair<Bus, Bus>(sub1, sub2), Duration.ZERO);
		FlexRay main = new FlexRay(EEsim);// mainComponents
		for (EEComponent component : mainComponents) {
			main.registerComponent(component);
		}
		new Bridge(EEsim, new ImmutablePair<Bus, Bus>(main, sub1), Duration.ZERO);
		return main;
	}

	private BusMessage createNregisterMessage(FlexRay flexray, Object message, int senderPos, int messageLength,
			int priority) {
		assertTrue(busEntryByOrdinal.size() > priority);

		List<EEComponent> connectedComponents = flexray.getConnectedComponents();
		assertTrue(senderPos >= 0 && senderPos < connectedComponents.size());

		BusMessage msg = new BusMessage(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
				connectedComponents.get(senderPos).getId(), flexray);
		flexray.registerMessage(msg);
		return msg;
	}

	private List<BusMessage> createNregisterMessages(FlexRay flexray) {
		List<BusMessage> msgs = new ArrayList<BusMessage>();

		List<EEComponent> connectedComponents = flexray.getConnectedComponents();

		// make sure ordering is deterministic => no two messages with same priority
		List<Integer> priorities = new ArrayList<Integer>();
		for (int j = 0; j < busEntryByOrdinal.size(); j++) {
			priorities.add(j);
		}

		Random rand = new Random();
		for (int j = 0; j < busEntryByOrdinal.size(); j++) {
			int senderPos = rand.nextInt(connectedComponents.size());
			int messageLength = rand.nextInt(1500);
			int priority = priorities.remove(rand.nextInt(priorities.size()));
			msgs.add(createNregisterMessage(flexray, j, senderPos, messageLength, priority));
		}
		return msgs;
	}
}
