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
//package simulation.EESimulator;
//
//import commons.controller.commons.BusEntry;
//import org.junit.Test;
//import simulation.bus.Bus;
//import simulation.bus.BusMessage;
//import simulation.bus.FlexRay;
//
//import java.lang.reflect.Array;
//import java.time.Instant;
//import java.util.ArrayList;
//import java.util.LinkedList;
//import java.util.List;
//
//import static junit.framework.TestCase.assertEquals;
//
//public class SimpleEESimulatorTest {
//
//    static List<BusMessage> messageList = new LinkedList<>();
//
//    public static void setProcessed(BusMessage message, EESimulatorTestComponent component) {
//        for (BusMessage msg: messageList) {
//            if(msg.getMessage().equals(message.getMessage())){
//                msg.setMessage(component.getID().toString() + " processed");
//            }
//
//        }
//    }
//
//    @Test
//    public void testSimulateNextTick(){
//        EESimulator simulator = new EESimulator(Instant.EPOCH);
//
//        Instant simulationTime = Instant.EPOCH.plusSeconds(10);
//
//        ArrayList<BusEntry> targetList = new ArrayList<>();
//        ArrayList<BusEntry> emptyList = new ArrayList<>();
//        targetList.add(BusEntry.ACTUATOR_ENGINE);
//        EESimulatorTestComponent comp1 = new EESimulatorTestComponent(simulator, targetList);
//        EESimulatorTestComponent transmitter = new EESimulatorTestComponent(simulator,emptyList);
//
//        FlexRay bus = new FlexRay(simulator);
//        bus.registerComponent(comp1);
//        bus.registerComponent(transmitter);
//
//        BusMessage message = new BusMessage("message", 100, BusEntry.ACTUATOR_ENGINE,
//                Instant.EPOCH.plusNanos(3), transmitter.getID(), bus);
//        messageList.add(message);
//
//        simulator.addEvent(message);
//        for (EEDiscreteEvent messages: simulator.getEventList()) {
//            System.out.println(messages.getEventType());
//            if(messages.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE) System.out.println(((BusMessage) messages).getMessage());
//            System.out.println(messages.getEventTime());
//        }
//
//        simulator.simulateNextTick(simulationTime);
//
//
//
//        assertEquals(comp1.getID().toString() + " processed", message.getMessage());
//    }
//
//
//}
