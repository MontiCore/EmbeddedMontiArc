import static commons.controller.commons.BusEntry.NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Stack;

import org.junit.BeforeClass;
import org.junit.Test;

import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;

import commons.controller.commons.BusEntry;

public class BusTest {
	
	private static EESimulator EEsim = new EESimulator();
	
	private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

	@BeforeClass
	public static void oneTimeSetUp() {
		for (BusEntry entry : BusEntry.values()) {
			busEntryByOrdinal.put(entry.ordinal(), entry);
		}
	}
	
	@Test
	public void testSetPath() {
		FlexRay flexray = createBusStructure();
		
		//local
		BusMessage c01 = createNregisterMessage(flexray, "c01", "TestComponent 0", "TestComponent 1", 127, 0);
		
		//one hop
		BusMessage c37 = createNregisterMessage(flexray, "c37", "TestComponent 3", "TestComponent 7", 127, 0);

		//two hop
		BusMessage c49 = createNregisterMessage(flexray, "c49", "TestComponent 4", "TestComponent 9", 127, 0);
		
		Iterable<Bus> buses = BusUtils.findConnectedBuses(flexray.connectedComponents);
		Iterator<Bus> it = buses.iterator();
		assertTrue(it.hasNext());
		Bus firstHop = it.next();
		assertTrue(!it.hasNext());
		
		buses = BusUtils.findConnectedBuses(firstHop.connectedComponents);
		it = buses.iterator();
		assertTrue(it.hasNext());
		Bus secondHop = it.next();
		assertTrue(!it.hasNext());
		
		assertEquals("TestComponent 1", c01.getNextHop().get().getID());
		assertTrue(c01.getPath().isEmpty());
		
		assertEquals(firstHop.getID(), c37.getNextHop().get().getID());
		assertEquals(1, c37.getPath().size());
		assertEquals("TestComponent 7", c37.getPath().get(0).getID());
		
		assertEquals(firstHop.getID(), c49.getNextHop().get().getID());
		assertEquals(2, c49.getPath().size());
		assertEquals(secondHop.getID(), c49.getPath().get(0).getID());
		assertEquals("TestComponent 9", c49.getPath().get(1).getID());
	}	
	
	@Test
	public void testRegisterEventAtSimulator() {
		FlexRay flexray = createBusStructure();

		//one hop
		BusMessage c05 = createNregisterMessage(flexray, "c05", "TestComponent 0", "TestComponent 5", 127, 0);

		//two hop
		BusMessage c08 = createNregisterMessage(flexray, "c08", "TestComponent 0", "TestComponent 8", 127, 0);
		
		Iterable<Bus> buses = BusUtils.findConnectedBuses(flexray.connectedComponents);
		Iterator<Bus> it = buses.iterator();
		assertTrue(it.hasNext());
		Bus firstHop = it.next();
		assertTrue(!it.hasNext());
		
		buses = BusUtils.findConnectedBuses(firstHop.connectedComponents);
		it = buses.iterator();
		assertTrue(it.hasNext());
		Bus secondHop = it.next();
		assertTrue(!it.hasNext());
		
		assertEquals(firstHop.getID(), c05.getNextHop().get().getID());
		assertEquals(firstHop.getID(), c08.getNextHop().get().getID());
		
		flexray.registerEventAtSimulator(c05);
		flexray.registerEventAtSimulator(c08);
		
		assertEquals("TestComponent 5", c05.getNextHop().get().getID());
		assertEquals(secondHop.getID(), c08.getNextHop().get().getID());
		
		flexray.registerEventAtSimulator(c08);
		assertEquals("TestComponent 8", c08.getNextHop().get().getID());
	}
	
	
	//TODO other bus impl.
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

		FlexRay sub2 = new FlexRay(EEsim, subComponents2);
		subComponents1.add(sub2);
		FlexRay sub1 = new FlexRay(EEsim, subComponents1);
		mainComponents.add(sub1);
		FlexRay main = new FlexRay(EEsim, mainComponents);
		return main;
	}

	private BusMessage createNregisterMessage(FlexRay flexray, Object message, String senderID, String receiverID,
			int messageLength, int priority) {
		Optional<EEComponent> sender = BusUtils.findComponentWithID(flexray.connectedComponents, senderID);
		assertTrue(sender.isPresent());
		Stack<Bus> workStack = new Stack<Bus>();
		workStack.push(flexray);
		Optional<EEComponent> receiver = Optional.empty();
		while(!workStack.isEmpty() && !receiver.isPresent()) {
			Bus cur = workStack.pop();
			receiver = BusUtils.findComponentWithID(cur.connectedComponents, receiverID);
			Iterable<Bus> buses = BusUtils.findConnectedBuses(cur.connectedComponents);
			for(Bus bus : buses) {
				workStack.push(bus);
			}
		}
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
}
