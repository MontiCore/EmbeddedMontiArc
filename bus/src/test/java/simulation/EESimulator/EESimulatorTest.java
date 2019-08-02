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
package simulation.EESimulator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.time.Instant;
import java.util.*;


import commons.controller.commons.BusEntry;
import org.junit.Test;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.*;

public class EESimulatorTest {


    static List<BusMessage> messageList = new LinkedList<>();

    public static void setProcessed(BusMessage message, EESimulatorTestComponent component) {
        for (BusMessage msg: messageList) {
            if(msg.getMessage().equals(message.getMessage())){
                msg.setMessage(component.getID().toString() + " processed");
            }

        }
    }

    @Test
    public void testSimulateNextTick() {


        EESimulator simulator = new EESimulator(Instant.EPOCH);

        Instant simTime = Instant.EPOCH;

        ArrayList<BusEntry> listenToList = new ArrayList<>();
        listenToList.add(BusEntry.ACTUATOR_ENGINE);
        EESimulatorTestComponent tesComp1 = new EESimulatorTestComponent(simulator, listenToList); //gets ACTUATOR_ENGINE
        listenToList.set(0, BusEntry.ACTUATOR_GEAR);
        EESimulatorTestComponent testComp2 = new EESimulatorTestComponent(simulator, listenToList); //gets ACTUATOR_GEAR
        listenToList.set(0, BusEntry.ACTUATOR_BRAKE);
        EESimulatorTestComponent testComp3 = new EESimulatorTestComponent(simulator, listenToList); //gets ACTUATOR_BRAKE
        listenToList.set(0, BusEntry.ACTUATOR_STEERING);
        listenToList.add(BusEntry.SENSOR_COMPASS);
        EESimulatorTestComponent testComp4 = new EESimulatorTestComponent(simulator, listenToList); //gets ACTUATOR_STEERING and SENSOR_COMPASS

        TestComponent transmitter = new TestComponent(simulator);


        FlexRay testBus = new FlexRay(simulator);
        testBus.registerComponent(tesComp1);
        testBus.registerComponent(testComp2);
        testBus.registerComponent(testComp3);
        testBus.registerComponent(testComp4);
        testBus.registerComponent(transmitter);

        BusMessage message1 = new BusMessage("messageOne", 100, BusEntry.ACTUATOR_ENGINE,
                Instant.EPOCH.plusSeconds(3), transmitter.getID(), testBus);
        messageList.add(message1);
        BusMessage message2 = new BusMessage("messageTwo", 200, BusEntry.ACTUATOR_GEAR,
                Instant.EPOCH.plusSeconds(32), transmitter.getID(), testBus);
        messageList.add(message2);
        BusMessage message3 = new BusMessage("messageThree", 20, BusEntry.ACTUATOR_BRAKE,
                Instant.EPOCH.plusSeconds(38), transmitter.getID(), testBus);
        messageList.add(message3);
        BusMessage message4 = new BusMessage("messageFour", 150, BusEntry.ACTUATOR_STEERING,
                Instant.EPOCH.plusSeconds(4), transmitter.getID(), testBus);
        messageList.add(message4);
        BusMessage message5 = new BusMessage("messageFive", 30, BusEntry.ACTUATOR_ENGINE,
                Instant.EPOCH.plusSeconds(15), transmitter.getID(), testBus);
        messageList.add(message5);
        BusMessage message6 = new BusMessage("messageSix", 120, BusEntry.SENSOR_COMPASS,
                Instant.EPOCH.plusSeconds(40), transmitter.getID(), testBus);
        messageList.add(message6);


        //start first Tick of the Simulation
        System.out.println("Run first tick");
        System.out.println("--------------");

        simulator.addEvent(message1);
        simulator.addEvent(message2);
        simulator.addEvent(message4);

        simulator.simulateNextTick(simTime);

        assertEquals(message1.getMessage(), "messageOne");
        assertEquals(message2.getMessage(), "messageTwo");
        assertEquals(message4.getMessage(), "messageFour");


        System.out.println("---------------");
        //simulate second tick
        //simTime.plusNanos(30);
        Instant simTimeTwo = Instant.EPOCH.plusSeconds(30);
        System.out.println("Run second tick");
        System.out.println("---------------");

        simulator.addEvent(message3);
        simulator.addEvent(message5);


        for (EEDiscreteEvent messages: simulator.getEventList()) {
            System.out.println(messages.getEventType());
            if(messages.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) System.out.println(((BusMessage) messages).getMessage());
            System.out.println(messages.getEventTime());
        }


        simulator.simulateNextTick(simTimeTwo);

        assertEquals(tesComp1.getID().toString() + " processed", message1.getMessage());
        assertEquals("messageTwo", message2.getMessage());
        assertEquals("messageThree", message3.getMessage());
        assertEquals(testComp4.getID().toString() + " processed", message4.getMessage());
        assertEquals(tesComp1.getID().toString() + " processed", message5.getMessage());

        System.out.println("---------------");
        //simulate third tick
        //simTime.plusNanos(30);
        Instant simTimeThree = Instant.EPOCH.plusSeconds(60);
        System.out.println("Run third tick");
        System.out.println("--------------");

        simulator.addEvent(message6);

        simulator.simulateNextTick(simTimeThree);

        assertEquals(message1.getMessage(), tesComp1.getID().toString() + " processed");
        assertEquals(message2.getMessage(), testComp2.getID().toString() + " processed");
        assertEquals(message3.getMessage(), testComp3.getID().toString() + " processed");
        assertEquals(message4.getMessage(), testComp4.getID().toString() + " processed");
        assertEquals(message5.getMessage(), tesComp1.getID().toString() + " processed");
        assertEquals(message6.getMessage(), testComp4.getID().toString() + " processed");


    }
}

//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//	/*
//	private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();
//
//	@BeforeClass
//	public static void oneTimeSetUp() {
//		for (BusEntry entry : BusEntry.values()) {
//			busEntryByOrdinal.put(entry.ordinal(), entry);
//		}
//	}
//
//	public void testSimulateNextTick() {
//		EESimulator sim = new EESimulator(Instant.EPOCH);
//		FlexRay flexray = createBusStructure(sim);
//
////		BusMessage c01 = createMessage(flexray, "c01", "TestComponent 0", "TestComponent 1", 127, 0);
////		BusMessage c02 = createMessage(flexray, "c02", "TestComponent 0", "TestComponent 1",
////				(254 + 516 + 254 + 200), 4);
////
////		BusMessage c11 = createMessage(flexray, "c11", "TestComponent 1", "TestComponent 1", 127, 0);
////		BusMessage c12 = createMessage(flexray, "c12", "TestComponent 1", "TestComponent 1", (254 + 254 + 500),
////				3);
////
////		BusMessage c21 = createMessage(flexray, "c21", "TestComponent 2", "TestComponent 1", 100, 0);
////		BusMessage c22 = createMessage(flexray, "c22", "TestComponent 2", "TestComponent 1", 100, 0);
////		BusMessage c23 = createMessage(flexray, "c23", "TestComponent 2", "TestComponent 1", 54, 0);
////
////		BusMessage c31 = createMessage(flexray, "c31", "TestComponent 3", "TestComponent 1", 100, 0);
////		BusMessage c32 = createMessage(flexray, "c32", "TestComponent 3", "TestComponent 1", 200, 1);
////		BusMessage c33 = createMessage(flexray, "c33", "TestComponent 3", "TestComponent 1", (154 + 816), 5);
////		BusMessage c34 = createMessage(flexray, "c34", "TestComponent 3", "TestComponent 1", 100, 6);
//	}
//
//
//	//TODO other Bus impl.
//	private FlexRay createBusStructure(EESimulator sim) {
//		List<EESimulator> mainComponents = new ArrayList<EESimulator>();
//		List<EESimulator> subComponents1 = new ArrayList<EESimulator>();
//		List<EESimulator> subComponents2 = new ArrayList<EESimulator>();
//		int i = 0;
//		for (; i < 5; i++) {
//			mainComponents.add(new TestComponent(sim, String.valueOf(i)));
//		}
//		for (; i < 8; i++) {
//			subComponents1.add(new TestComponent(sim, String.valueOf(i)));
//		}
//		for (; i < 15; i++) {
//			subComponents2.add(new TestComponent(sim, String.valueOf(i)));
//		}
//
//		List<BusEntry> messages = new ArrayList<BusEntry>();
//		messages.add(BusEntry.CONSTANT_WHEELBASE);
//
//		FlexRay sub2 = new FlexRay(sim);//subComponents2
//		for(EEComponent component: subComponents2){
//			sub2.registerComponent(component, messages);
//		}
//
//		subComponents1.add(sub2);
//		FlexRay sub1 = new FlexRay(sim);//subComponents1
//		for(EEComponent component: subComponents1){
//			sub1.registerComponent(component, messages);
//		}
//
//		mainComponents.add(sub1);
//		FlexRay main = new FlexRay(sim);//mainComponents
//		for(EEComponent component: mainComponents){
//			main.registerComponent(component, messages);
//		}
//
//		return main;
//	}
//
//	private BusMessage createMessage(FlexRay flexray, Object message, String senderID, String receiverID,
//			int messageLength, int priority, Instant eventTime) {
//		Optional<EESimulator> sender = BusUtils.findComponentWithID(flexray.connectedComponents, senderID);
//		assertTrue(sender.isPresent());
//		Optional<EESimulator> receiver = BusUtils.findComponentWithID(flexray.connectedComponents, receiverID);
//		assertTrue(receiver.isPresent());
//
//		assertTrue(busEntryByOrdinal.size() > priority);
//
//		BusMessage msg = new BusMessage(message, messageLength, busEntryByOrdinal.get(priority), eventTime,
//				MessageType.SEND, receiver.get());
//		msg.setControllerID(sender.get().getID());
//		return msg;
//	}*/
//}
//
