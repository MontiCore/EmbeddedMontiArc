package simulation.bus;
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

import static commons.controller.commons.BusEntry.NAVIGATION_DETAILED_PATH_WITH_MAX_STEERING_ANGLE;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import java.util.Stack;


import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.BeforeClass;
import org.junit.Test;

import commons.controller.commons.BusEntry;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.TestComponent;
import simulation.bus.*;

public class BusTest {
	
	private static EESimulator EEsim = new EESimulator(Instant.EPOCH);
	
	private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

	@BeforeClass
	public static void oneTimeSetUp() {
		for (BusEntry entry : BusEntry.values()) {
			busEntryByOrdinal.put(entry.ordinal(), entry);
		}
	}
	
	
//	@Test
//	public void testRegisterEventAtSimulator() {
//		FlexRay flexray = createBusStructure();
//		
//		//one hop
//		BusMessage c05 = createNregisterMessage(flexray, "c05", "TestComponent 0", "TestComponent 5", 127, 0);
//
//		//two hop
//		BusMessage c08 = createNregisterMessage(flexray, "c08", "TestComponent 0", "TestComponent 8", 127, 0);
//		
//		Iterable<Bus> buses = BusUtils.findConnectedBuses(flexray.connectedComponents);
//		Iterator<Bus> it = buses.iterator();
//		assertTrue(it.hasNext());
//		Bus firstHop = it.next();
//		assertTrue(!it.hasNext());
//		
//		buses = BusUtils.findConnectedBuses(firstHop.connectedComponents);
//		it = buses.iterator();
//		assertTrue(it.hasNext());
//		Bus secondHop = it.next();
//		assertTrue(!it.hasNext());
//		
//		assertEquals(firstHop.getID(), c05.getNextHop().get().getID());
//		assertEquals(firstHop.getID(), c08.getNextHop().get().getID());
//		
//		flexray.registerMessageAtSimulator(c05);
//		flexray.registerMessageAtSimulator(c08);
//		
//		assertEquals("TestComponent 5", c05.getNextHop().get().getID());
//		assertEquals(secondHop.getID(), c08.getNextHop().get().getID());
//		
//		flexray.registerMessageAtSimulator(c08);
//		assertEquals("TestComponent 8", c08.getNextHop().get().getID());
//	}
	
	
	//TODO other bus impl.
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

	private BusMessage createNregisterMessage(FlexRay flexray, Object message, int senderPos, int receiverPos,
			int messageLength, int priority) {
		assertTrue(busEntryByOrdinal.size() > priority);

		List<EEComponent> connectedComponents = flexray.getConnectedComponents();
		assertTrue(senderPos >= 0 && senderPos < connectedComponents.size());
		assertTrue(receiverPos >= 0 && receiverPos < connectedComponents.size());

		BusMessage msg = new BusMessage(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
				connectedComponents.get(senderPos).getID(), connectedComponents.get(receiverPos));
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
			int receiverPos = rand.nextInt(connectedComponents.size());
			int messageLength = rand.nextInt(1500);
			int priority = priorities.remove(rand.nextInt(priorities.size()));
			msgs.add(createNregisterMessage(flexray, j, senderPos, receiverPos, messageLength, priority));
		}
		return msgs;
	}
}
