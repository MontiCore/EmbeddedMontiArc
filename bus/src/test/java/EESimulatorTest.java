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
import static org.junit.Assert.assertTrue;

import java.time.Instant;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.junit.BeforeClass;

import commons.controller.commons.BusEntry;

public class EESimulatorTest {

	/*
	public void testSimulateNextTick(){

		EESimulator simulator = new EESimulator(Instant.EPOCH);




	}























	/*
	private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

	@BeforeClass
	public static void oneTimeSetUp() {
		for (BusEntry entry : BusEntry.values()) {
			busEntryByOrdinal.put(entry.ordinal(), entry);
		}
	}

	public void testSimulateNextTick() {
		EESimulator sim = new EESimulator(Instant.EPOCH);
		FlexRay flexray = createBusStructure(sim);

//		BusMessage c01 = createMessage(flexray, "c01", "TestComponent 0", "TestComponent 1", 127, 0);
//		BusMessage c02 = createMessage(flexray, "c02", "TestComponent 0", "TestComponent 1",
//				(254 + 516 + 254 + 200), 4);
//
//		BusMessage c11 = createMessage(flexray, "c11", "TestComponent 1", "TestComponent 1", 127, 0);
//		BusMessage c12 = createMessage(flexray, "c12", "TestComponent 1", "TestComponent 1", (254 + 254 + 500),
//				3);
//
//		BusMessage c21 = createMessage(flexray, "c21", "TestComponent 2", "TestComponent 1", 100, 0);
//		BusMessage c22 = createMessage(flexray, "c22", "TestComponent 2", "TestComponent 1", 100, 0);
//		BusMessage c23 = createMessage(flexray, "c23", "TestComponent 2", "TestComponent 1", 54, 0);
//
//		BusMessage c31 = createMessage(flexray, "c31", "TestComponent 3", "TestComponent 1", 100, 0);
//		BusMessage c32 = createMessage(flexray, "c32", "TestComponent 3", "TestComponent 1", 200, 1);
//		BusMessage c33 = createMessage(flexray, "c33", "TestComponent 3", "TestComponent 1", (154 + 816), 5);
//		BusMessage c34 = createMessage(flexray, "c34", "TestComponent 3", "TestComponent 1", 100, 6);
	}
	

	//TODO other Bus impl.
	private FlexRay createBusStructure(EESimulator sim) {
		List<EEComponent> mainComponents = new ArrayList<EEComponent>();
		List<EEComponent> subComponents1 = new ArrayList<EEComponent>();
		List<EEComponent> subComponents2 = new ArrayList<EEComponent>();
		int i = 0;
		for (; i < 5; i++) {
			mainComponents.add(new TestComponent(sim, String.valueOf(i)));
		}
		for (; i < 8; i++) {
			subComponents1.add(new TestComponent(sim, String.valueOf(i)));
		}
		for (; i < 15; i++) {
			subComponents2.add(new TestComponent(sim, String.valueOf(i)));
		}

		FlexRay sub2 = new FlexRay(sim, subComponents2);
		subComponents1.add(sub2);
		FlexRay sub1 = new FlexRay(sim, subComponents1);
		mainComponents.add(sub1);
		FlexRay main = new FlexRay(sim, mainComponents);
		return main;
	}

	private BusMessage createMessage(FlexRay flexray, Object message, String senderID, String receiverID,
			int messageLength, int priority, Instant eventTime) {
		Optional<EEComponent> sender = BusUtils.findComponentWithID(flexray.connectedComponents, senderID);
		assertTrue(sender.isPresent());
		Optional<EEComponent> receiver = BusUtils.findComponentWithID(flexray.connectedComponents, receiverID);
		assertTrue(receiver.isPresent());

		assertTrue(busEntryByOrdinal.size() > priority);

		BusMessage msg = new BusMessage(message, messageLength, busEntryByOrdinal.get(priority), eventTime,
				MessageType.SEND, receiver.get());
		msg.setControllerID(sender.get().getID());
		return msg;
	}*/
}
