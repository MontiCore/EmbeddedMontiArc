/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.bus;

import static de.rwth.montisim.commons.controller.commons.BusEntry.NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.time.Duration;
import java.time.Instant;
import java.util.*;


import de.rwth.montisim.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.BeforeClass;
import org.junit.Test;

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

        flexRay.setOperationMode(FlexRayOperationMode.MAX_DATA_RATE);
        long expectedNs = (int) Math.ceil((FlexRay.MAX_SLOT_SIZE * 8 * 1000000) / ((double) 20));
        assertEquals(expectedNs, flexRay.getSlotDuration().toNanos());

        flexRay.setOperationMode(FlexRayOperationMode.REDUNDANCY);
        expectedNs = (int) Math.ceil((FlexRay.MAX_SLOT_SIZE * 8 * 1000000) / ((double) 10));
        System.out.println(flexRay.getSlotDuration().toMillis());
        assertEquals(expectedNs, flexRay.getSlotDuration().toNanos());
    }

    @Test
    public void testCycleTime() {
        FlexRay flexRay = createBusStructure();

        long expectedNs = flexRay.getSlotDuration().toNanos()
                * (flexRay.getConnectedComponents().size() + FlexRay.DYNAMIC_SLOTS);
        assertEquals(expectedNs, flexRay.getCycleDuration().toNanos());
    }

    @Test(expected = IllegalArgumentException.class)
    public void testRegisterMessageFromInvalidController() {
        Bus bus = createBusStructure();

        EEComponent comp = bus.getConnectedComponents().get(0);
        UUID invalidID = UUID.randomUUID();
        String msg = "FromIllegalController";
        BusMessageEvent illegalControllerMsg = new BusMessageEvent(msg, msg.length(),
                NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, invalidID, comp);
        bus.registerMessage(illegalControllerMsg);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testRegisterMessageToInvalidController() {
        Bus bus = createBusStructure();

        EEComponent comp = bus.getConnectedComponents().get(0);
        EEComponent illegalController = new TestComponent(EEsim);

        String msg = "ToIllegalController";
        BusMessageEvent illegalControllerMsg = new BusMessageEvent(msg, msg.length(),
                NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE, Instant.EPOCH, comp.getId(), illegalController);
        bus.registerMessage(illegalControllerMsg);
    }

    @Test
    public void testGetNextDynamicMessage() throws NoSuchMethodException, SecurityException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException {
        FlexRay flexray = createBusStructure();

        Method getNextDymaicMessage = FlexRay.class.getDeclaredMethod("getNextDynamicMessage");
        getNextDymaicMessage.setAccessible(true);
        BusMessageEvent empty = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(empty == null);

        BusMessageEvent c01 = createNregisterMessage(flexray, "c01", 0, 1, 0);
        BusMessageEvent msg = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(msg != null);
        assertEquals(msg, c01);

        BusMessageEvent c02 = createNregisterMessage(flexray, "c02", 0, 254, 1);
        msg = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(msg != null);
        assertEquals(msg, c02);

        BusMessageEvent c11 = createNregisterMessage(flexray, "c11", 1, 127, 2);
        msg = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(msg != null);
        assertEquals(msg, c11);

        BusMessageEvent c12 = createNregisterMessage(flexray, "c12", 1, 127, 3);
        msg = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(msg != null);
        assertEquals(msg, c12);

        createNregisterMessage(flexray, "", 1, 1, 0);
        msg = (BusMessageEvent) getNextDymaicMessage.invoke(flexray);
        assertTrue(msg != null);
        assertEquals(msg, c12);
    }

    @Test
    public void testFillStaticSegment() throws NoSuchMethodException, SecurityException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException {
        FlexRay flexray = createBusStructure();
        createNregisterMessages(flexray);
        Map<Integer, List<BusMessageEvent>> msgsByEndCycle = new HashMap<Integer, List<BusMessageEvent>>();
        for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
            BusMessageEvent msgArray[] = controllerMsgs.toArray(new BusMessageEvent[controllerMsgs.size()]);
            Arrays.parallelSort(msgArray, controllerMsgs.comparator());
            int totalBytes = 0;
            for (int i = 0; i < controllerMsgs.size(); i++) {
                BusMessageEvent cur = msgArray[i];
                totalBytes += cur.getMessageLen();
                int cycle = totalBytes / FlexRay.MAX_SLOT_PAYLOAD;
                if ((totalBytes % FlexRay.MAX_SLOT_PAYLOAD) != 0) {
                    cycle++;
                }
                System.out.println("Total bytes: " + totalBytes + " Cycle: " + cycle);
                List<BusMessageEvent> finishedMsgs = msgsByEndCycle.getOrDefault(cycle, new ArrayList<BusMessageEvent>());
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

        List<Map.Entry<Integer, List<BusMessageEvent>>> entries = new ArrayList<>(msgsByEndCycle.entrySet());
        Collections.sort(entries, (e1, e2) -> e1.getKey() - e2.getKey());
        for (Map.Entry<Integer, List<BusMessageEvent>> entry : entries) {
            System.out.println("-----------------------");
            for (Map.Entry<UUID, Boolean> first : firstByControllerId.entrySet()) {
                first.setValue(true);
            }

            for (BusMessageEvent msg : entry.getValue()) {
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
            for (BusMessageEvent msg : entry.getValue()) {
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
        List<BusMessageEvent> msgs = createNregisterMessages(flexray);

        Map<Integer, List<BusMessageEvent>> msgsByEndCycle = new HashMap<Integer, List<BusMessageEvent>>();
        BusMessageEvent msgArray[] = msgs.toArray(new BusMessageEvent[msgs.size()]);
        Arrays.parallelSort(msgArray, new BusMessageComparatorIdDesc());
        int totalBytes = 0;
        int bytesPerCycle = FlexRay.DYNAMIC_SLOTS * FlexRay.MAX_SLOT_PAYLOAD;
        for (int i = 0; i < msgArray.length; i++) {
            BusMessageEvent cur = msgArray[i];
            totalBytes += cur.getMessageLen();
            int cycle = totalBytes / bytesPerCycle;
            if ((totalBytes % bytesPerCycle) != 0) {
                cycle++;
            }
            System.out.println("Total bytes: " + totalBytes + " Cycle: " + cycle);
            List<BusMessageEvent> finishedMsgs = msgsByEndCycle.getOrDefault(cycle, new ArrayList<BusMessageEvent>());
            finishedMsgs.add(cur);
            msgsByEndCycle.put(cycle, finishedMsgs);
        }

        int curCycle = 0;
        boolean first = true;
        int transmittedBytes = 0;

        List<Map.Entry<Integer, List<BusMessageEvent>>> entries = new ArrayList<>(msgsByEndCycle.entrySet());
        Collections.sort(entries, (e1, e2) -> e1.getKey() - e2.getKey());
        for (Map.Entry<Integer, List<BusMessageEvent>> entry : entries) {
            System.out.println("-----------------------");
            first = true;

            for (BusMessageEvent msg : entry.getValue()) {
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

            Method fillDynamicSegment = FlexRay.class.getDeclaredMethod("fillDynamicSegment", Instant.class);
            fillDynamicSegment.setAccessible(true);
            for (; curCycle < entry.getKey(); curCycle++) {
                fillDynamicSegment.invoke(flexray, Instant.EPOCH);
                transmittedBytes += bytesPerCycle;
            }

            for (BusMessageEvent msg : entry.getValue()) {
                System.out.println("Expected to finish: " + msg.getMessage().toString());
                assertTrue(msg.isTransmitted());
                transmittedBytes -= msg.getMessageLen();
            }
        }
    }

    @Test
    public void testGetNextFinishTime() {
        FlexRay flexray = createBusStructure();

        BusMessageEvent dynamic = createNregisterMessage(flexray, "finished in last dynamic slot", 0, ((FlexRay.MAX_SLOT_PAYLOAD * (FlexRay.DYNAMIC_SLOTS)) + FlexRay.MAX_SLOT_PAYLOAD - 3), 0);
        BusMessageEvent copy = new BusMessageEvent(dynamic);
        Instant finishTime = flexray.getNextFinishTime();
        assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
        Instant minFinishTime = Instant.EPOCH.plus(flexray.getStaticSegmentDuration())
                .plus(flexray.getSlotDuration().multipliedBy(3));
        assertTrue(minFinishTime.isBefore(finishTime));
        assertTrue(minFinishTime.plus(flexray.getSlotDuration()).isAfter(finishTime));
        flexray.simulateUntil(finishTime);
        assertTrue(dynamic.isTransmitted());
        assertEquals(finishTime, dynamic.getFinishTime());
        assertEquals(finishTime, flexray.getCurrentTime());

        dynamic = createNregisterMessage(flexray, "finished at end of last dynamic slot", 0, 3, 0);
        copy = new BusMessageEvent(dynamic);
        finishTime = flexray.getNextFinishTime();
        assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
        assertEquals(Instant.EPOCH.plus(flexray.getCycleDuration()), finishTime);
        flexray.simulateUntil(finishTime);
        assertTrue(dynamic.isTransmitted());
        assertEquals(finishTime, dynamic.getFinishTime());

        //start from incomplete dynamic slot
        dynamic = createNregisterMessage(flexray, "finished in last dynamic slot", 0, ((FlexRay.MAX_SLOT_PAYLOAD * (FlexRay.DYNAMIC_SLOTS)) + FlexRay.MAX_SLOT_PAYLOAD - 5), 0);
        copy = new BusMessageEvent(dynamic);
        finishTime = flexray.getNextFinishTime();
        assertEquals(copy.getTransmittedBytes(), dynamic.getTransmittedBytes());
        minFinishTime = Instant.EPOCH.plus(flexray.getCycleDuration()).plus(flexray.getStaticSegmentDuration())
                .plus(flexray.getSlotDuration().multipliedBy(3));
        assertTrue(minFinishTime.isBefore(finishTime));
        assertTrue(minFinishTime.plus(flexray.getSlotDuration()).isAfter(finishTime));
        flexray.simulateUntil(finishTime);
        assertTrue(dynamic.isTransmitted());
        assertEquals(finishTime, dynamic.getFinishTime());

        // create message that takes up dynamic slots
        dynamic = createNregisterMessage(flexray, "dynamic", 0, Integer.MAX_VALUE, 20);

        Random rand = new Random();
        Instant lastFinishedCycle = Instant.EPOCH.plus(flexray.getCycleDuration().multipliedBy(2));
        for (int i = 2; i < flexray.getConnectedComponents().size(); i++) {
            System.out.println("--------------new Message-----------------");
            int slotNumber = 0;
            int messageLen = 0;
            if (i == 2) {
                messageLen = FlexRay.MAX_SLOT_PAYLOAD;
            } else {
                messageLen = rand.nextInt(FlexRay.MAX_SLOT_PAYLOAD) + 1;
            }
            //start from incomplete static segment
            BusMessageEvent controller = createNregisterMessage(flexray, "static", i - 1, messageLen, 0);
            for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
                if (!controllerMsgs.isEmpty() && controllerMsgs.peek() != dynamic) {
                    break;
                }
                slotNumber++;
            }
            copy = new BusMessageEvent(controller);
            finishTime = flexray.getNextFinishTime();
            assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());
            if (lastFinishedCycle.plus(flexray.getCycleDuration()).isBefore(finishTime)) {
                lastFinishedCycle = lastFinishedCycle.plus(flexray.getCycleDuration());
            }
            minFinishTime = lastFinishedCycle.plus(flexray.getSlotDuration().multipliedBy(slotNumber));
            System.out.println("Min finish time: " + minFinishTime + " max finish time " + minFinishTime.plus(flexray.getSlotDuration()) + " actual finish time " + finishTime);
            assertTrue(minFinishTime.isBefore(finishTime));
            assertTrue(!minFinishTime.plus(flexray.getSlotDuration()).isBefore(finishTime));
            flexray.simulateUntil(finishTime);
            assertTrue(controller.isTransmitted());
            assertEquals(finishTime, controller.getFinishTime());

            slotNumber = 0;
            controller = createNregisterMessage(flexray, "static", i, messageLen, 0);
            for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
                if (!controllerMsgs.isEmpty() && controllerMsgs.peek() != dynamic) {
                    break;
                }
                slotNumber++;
            }
            copy = new BusMessageEvent(controller);
            finishTime = flexray.getNextFinishTime();
            assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());
            if (lastFinishedCycle.plus(flexray.getCycleDuration()).isBefore(finishTime)) {
                lastFinishedCycle = lastFinishedCycle.plus(flexray.getCycleDuration());
            }
            minFinishTime = lastFinishedCycle.plus(flexray.getSlotDuration().multipliedBy(slotNumber));
            assertTrue(minFinishTime.isBefore(finishTime));
            assertTrue(!minFinishTime.plus(flexray.getSlotDuration()).isBefore(finishTime));
            flexray.simulateUntil(finishTime);
            assertTrue(controller.isTransmitted());
            assertEquals(finishTime, controller.getFinishTime());

            if (messageLen != FlexRay.MAX_SLOT_PAYLOAD) {
                controller = createNregisterMessage(flexray, "static", i, 16 - messageLen, 0);
                copy = new BusMessageEvent(controller);
                finishTime = flexray.getNextFinishTime();
                assertEquals(copy.getTransmittedBytes(), controller.getTransmittedBytes());
            }
            assertEquals(minFinishTime.plus(flexray.getSlotDuration()), finishTime);
            flexray.simulateUntil(finishTime);
            assertTrue(controller.isTransmitted());
            assertEquals(finishTime, controller.getFinishTime());
        }
    }

    @Test
    public void testGetNextFinishTimeLongMsgs() {

        FlexRay flexRay = createBusStructure();
        BusMessageEvent c00 = createNregisterMessage(flexRay, 33, 0, 741, BusEntry.ENVIRONMENT.ordinal());
        BusMessageEvent c10 = createNregisterMessage(flexRay, 17, 1, 183, BusEntry.COMPUTERVISION_TRACKED_CARS.ordinal());
        BusMessageEvent c20 = createNregisterMessage(flexRay, 36, 2, 128, BusEntry.COMPUTERVISION_DETECTED_LANES.ordinal());
        BusMessageEvent c21 = createNregisterMessage(flexRay, 36, 2, 1407, BusEntry.COMPUTERVISION_DETECTED_PEDESTRIANS.ordinal());
        BusMessageEvent c30 = createNregisterMessage(flexRay, 50, 3, 548, BusEntry.SIMULATION_DELTA_TIME.ordinal());
        BusMessageEvent c40 = createNregisterMessage(flexRay, 41, 4, 421, BusEntry.ACTUATOR_GEAR.ordinal());
        BusMessageEvent c50 = createNregisterMessage(flexRay, 14, 5, 1161, BusEntry.VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY.ordinal());

        Instant finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c20.isTransmitted());
        assertEquals(finishTime, c20.getFinishTime());

        System.out.println("-------------------------------");
        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c00.isTransmitted());
        assertEquals(finishTime, c00.getFinishTime());

        flexRay = createBusStructure();
        BusMessageEvent init = createNregisterMessage(flexRay, 42, 0, 32, BusEntry.COMPUTERVISION_VANISHING_POINT.ordinal());

        System.out.println("-------------------------------");
        System.out.println("-------------------------------");

        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(init.isTransmitted());
        assertEquals(finishTime, init.getFinishTime());

        c00 = createNregisterMessage(flexRay, 42, 0, 450, BusEntry.COMPUTERVISION_VANISHING_POINT.ordinal());
        c10 = createNregisterMessage(flexRay, 36, 1, 899, BusEntry.SENSOR_LANE.ordinal());
        c20 = createNregisterMessage(flexRay, 44, 2, 1484, BusEntry.ACTUATOR_THROTTLE_CURRENT.ordinal());
        c30 = createNregisterMessage(flexRay, 48, 3, 355, BusEntry.ACTUATOR_GEAR_CURRENT.ordinal());
        c40 = createNregisterMessage(flexRay, 16, 4, 412, BusEntry.ACTUATOR_STEERING.ordinal());
        c50 = createNregisterMessage(flexRay, 1, 5, 163, BusEntry.COMPUTERVISION_DETECTED_CARS.ordinal());

        System.out.println("-------------------------------");

        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c50.isTransmitted());
        assertEquals(finishTime, c50.getFinishTime());

        System.out.println("-------------------------------");
        System.out.println("-------------------------------");

        flexRay = createBusStructure();
        c00 = createNregisterMessage(flexRay, 33, 0, 1183, BusEntry.COMPUTERVISION_DETECTED_LANES.ordinal());
        c10 = createNregisterMessage(flexRay, 14, 1, 1102, BusEntry.SIMULATION_DELTA_TIME.ordinal());
        BusMessageEvent c11 = createNregisterMessage(flexRay, 4, 1, 653, BusEntry.ACTUATOR_THROTTLE.ordinal());
        c20 = createNregisterMessage(flexRay, 11, 2, 315, BusEntry.VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY.ordinal());
        c21 = createNregisterMessage(flexRay, 2, 2, 699, BusEntry.VEHICLE_MAX_TEMPORARY_ALLOWED_VELOCITY.ordinal());
        c30 = createNregisterMessage(flexRay, 29, 3, 1138, BusEntry.COMPUTERVISION_DETECTED_SIGNS.ordinal());
        c40 = createNregisterMessage(flexRay, 30, 4, 1148, BusEntry.ACTUATOR_THROTTLE_CURRENT.ordinal());
        c50 = createNregisterMessage(flexRay, 25, 5, 495, BusEntry.COMPUTERVISION_DETECTED_PEDESTRIANS.ordinal());


        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c10.isTransmitted());
        assertEquals(finishTime, c10.getFinishTime());

        System.out.println("-------------------------------");

        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c20.isTransmitted());
        assertEquals(finishTime, c20.getFinishTime());

        System.out.println("-------------------------------");

        finishTime = flexRay.getNextFinishTime();
        flexRay.simulateUntil(finishTime);
        assertEquals(finishTime, flexRay.getCurrentTime());
        assertTrue(c21.isTransmitted());
        assertEquals(finishTime, c21.getFinishTime());
    }

    @Test
    public void testGetNextFinishTimeRandomized() {
        FlexRay flexray = createBusStructure();
        List<BusMessageEvent> msgs = createNregisterMessages(flexray);
        int i = 0;
        for (; i < 2 && !msgs.isEmpty(); i++) {
            List<BusMessageEvent> firstMsgs = new ArrayList<BusMessageEvent>();
            for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
                PriorityQueue<BusMessageEvent> copy = new PriorityQueue<BusMessageEvent>(controllerMsgs);
                BusMessageEvent cur = copy.poll();
                while (cur != null && cur.isTransmitted()) {
                    cur = copy.poll();
                }
                if (cur != null) {
                    firstMsgs.add(cur);
                }
            }
            System.out.println("--------------New Message-------------");
            System.out.println("--------------" + msgs.size() + " remaining msgs-------------");
            Instant nextFinishTime = flexray.getNextFinishTime();
            System.out.println(nextFinishTime);
            flexray.simulateUntil(nextFinishTime);
            assertEquals(nextFinishTime, flexray.getCurrentTime());

            boolean transmitted = false;
            for (BusMessageEvent msg : firstMsgs) {
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
    public void testSimulateUntilRandomized() throws NoSuchMethodException, SecurityException, IllegalAccessException,
            IllegalArgumentException, InvocationTargetException {
        FlexRay flexray = createBusStructure();
        createNregisterMessages(flexray);

        PriorityQueue<BusMessageEvent> firstMsgs = new PriorityQueue<BusMessageEvent>(new BusMessageComparatorIdDesc());
        for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
            if (!controllerMsgs.isEmpty()) {
                firstMsgs.add(controllerMsgs.peek());
            }
        }
        Method simulateUntil = FlexRay.class.getDeclaredMethod("simulateUntil", Instant.class);
        simulateUntil.setAccessible(true);
        while (!firstMsgs.isEmpty()) {
            BusMessageEvent cur = firstMsgs.poll();
            // transmitted during static and dynamic segments
            int msgCycles = (int) Math.ceil(
                    cur.getRemainingBytes() / ((double) (FlexRay.MAX_SLOT_PAYLOAD * (FlexRay.DYNAMIC_SLOTS + 1))));
            simulateUntil.invoke(flexray,
                    flexray.getCurrentTime().plus(flexray.getCycleDuration().multipliedBy(msgCycles)));
            assertTrue(cur.isTransmitted());

            firstMsgs.clear();
            for (PriorityQueue<BusMessageEvent> controllerMsgs : flexray.getMessagesByControllerId().values()) {
                if (!controllerMsgs.isEmpty()) {
                    firstMsgs.add(controllerMsgs.peek());
                }
            }
        }
    }

    @Test
    public void testSimulateUntil() throws IllegalAccessException, IllegalArgumentException, InvocationTargetException,
            NoSuchMethodException, SecurityException {
        FlexRay flexray = createBusStructure();

        // finish in third cycle (static)
        BusMessageEvent c01 = createNregisterMessage(flexray, "c01", 0, FlexRay.MAX_SLOT_PAYLOAD, 0);
        // finish in second cycle (first cycle 1 dynamic slot, second cycle 0.5 dynamic slots)
        BusMessageEvent c02 = createNregisterMessage(flexray, "c02", 0, (FlexRay.MAX_SLOT_PAYLOAD * 3) + FlexRay.MAX_SLOT_PAYLOAD / 2, 4);

        // finish in third cycle (static)
        BusMessageEvent c11 = createNregisterMessage(flexray, "c11", 1, FlexRay.MAX_SLOT_PAYLOAD - 5, 0);
        // finish in second cycle (dynamic 3.5 slots)
        BusMessageEvent c12 = createNregisterMessage(flexray, "c12", 1, ((FlexRay.MAX_SLOT_PAYLOAD * 5) + (FlexRay.MAX_SLOT_PAYLOAD / 2)), 3);

        // finish in first cycle (static)
        BusMessageEvent c21 = createNregisterMessage(flexray, "c21", 2, FlexRay.MAX_SLOT_PAYLOAD / 3, 0);
        // finish in first cycle (static)
        BusMessageEvent c22 = createNregisterMessage(flexray, "c22", 2, FlexRay.MAX_SLOT_PAYLOAD / 3, 0);
        // finish in first cycle (static)
        BusMessageEvent c23 = createNregisterMessage(flexray, "c23", 2, FlexRay.MAX_SLOT_PAYLOAD / 3, 0);

        // finish in third cycle (static)
        BusMessageEvent c31 = createNregisterMessage(flexray, "c31", 3, FlexRay.MAX_SLOT_PAYLOAD, 0);
        // finish in second cycle (static)
        BusMessageEvent c32 = createNregisterMessage(flexray, "c32", 3, (FlexRay.MAX_SLOT_PAYLOAD / 3), 1);
        // finish in first cycle (first cycle 3 dynamic slots)
        BusMessageEvent c33 = createNregisterMessage(flexray, "c33", 3, (10 + (FlexRay.MAX_SLOT_PAYLOAD * 3)), 5);
        // finish in first cycle (static)
        BusMessageEvent c34 = createNregisterMessage(flexray, "c34", 3, FlexRay.MAX_SLOT_PAYLOAD - 10, 6);

        long slotSizeNs = flexray.getSlotDuration().toNanos();
        long totalStaticSegmentSizeNs = Math.toIntExact(flexray.getStaticSegmentDuration().toNanos());
        long cycleTimeNs = flexray.getCycleDuration().toNanos();

        Method simulateUntil = FlexRay.class.getDeclaredMethod("simulateUntil", Instant.class);
        simulateUntil.setAccessible(true);

        Instant currentTime = Instant.EPOCH;
        currentTime = currentTime.plusNanos(flexray.getCycleDuration().dividedBy(2).toNanos());
        simulateUntil.invoke(flexray, currentTime);
        assertEquals(currentTime, flexray.getCurrentTime());

        currentTime = currentTime.plusNanos(flexray.getCycleDuration().dividedBy(3).toNanos());
        simulateUntil.invoke(flexray, currentTime);
        assertEquals(currentTime, flexray.getCurrentTime());

        currentTime = currentTime.plusNanos(flexray.getCycleDuration().dividedBy(3).toNanos());
        simulateUntil.invoke(flexray, currentTime);
        assertEquals(currentTime, flexray.getCurrentTime());

        currentTime = currentTime.plusNanos(flexray.getCycleDuration().dividedBy(2).toNanos());
        simulateUntil.invoke(flexray, currentTime);
        assertEquals(currentTime, flexray.getCurrentTime());

        currentTime = currentTime.plusNanos(flexray.getCycleDuration().dividedBy(3).toNanos());
        simulateUntil.invoke(flexray, currentTime);
        assertEquals(currentTime, flexray.getCurrentTime());

        currentTime = currentTime.plusNanos(flexray.getCycleDuration().toNanos());
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
        assertTrue((cycleTimeNs + totalStaticSegmentSizeNs) < Duration
                .between(Instant.EPOCH, c02.getFinishTime()).toNanos());
        System.out.println("Expected: " + (cycleTimeNs + totalStaticSegmentSizeNs + slotSizeNs) + "; Actual: "
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

        new Bridge(new ImmutablePair<Bus, Bus>(sub1, sub2), Duration.ZERO);
        FlexRay main = new FlexRay(EEsim);// mainComponents
        for (EEComponent component : mainComponents) {
            main.registerComponent(component);
        }
        new Bridge(new ImmutablePair<Bus, Bus>(main, sub1), Duration.ZERO);
        return main;
    }

    private BusMessageEvent createNregisterMessage(FlexRay flexray, Object message, int senderPos, int messageLength,
                                                   int priority) {
        assertTrue(busEntryByOrdinal.size() > priority);

        List<EEComponent> connectedComponents = flexray.getConnectedComponents();
        assertTrue(senderPos >= 0 && senderPos < connectedComponents.size());

        BusMessageEvent msg = new BusMessageEvent(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
                connectedComponents.get(senderPos).getId(), flexray);
        flexray.registerMessage(msg);
        return msg;
    }

    private List<BusMessageEvent> createNregisterMessages(FlexRay flexray) {
        List<BusMessageEvent> msgs = new ArrayList<BusMessageEvent>();

        List<EEComponent> connectedComponents = flexray.getConnectedComponents();

        // make sure ordering is deterministic => no two messages with same priority
        List<Integer> priorities = new ArrayList<Integer>();
        for (int j = 0; j < busEntryByOrdinal.size(); j++) {
            priorities.add(j);
        }

        Random rand = new Random();
        for (int j = 0; j < busEntryByOrdinal.size(); j++) {
            int senderPos = rand.nextInt(connectedComponents.size());
            int messageLength = rand.nextInt(1499) + 1;
            int priority = priorities.remove(rand.nextInt(priorities.size()));
            msgs.add(createNregisterMessage(flexray, j, senderPos, messageLength, priority));
        }
        return msgs;
    }
}
