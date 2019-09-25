/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/**
* (c) https://github.com/MontiCore/monticore
*
* The license generally applicable for this project
* can be found under https://github.com/MontiCore/monticore.
*/

package simulation.bus;

import org.junit.BeforeClass;
import org.junit.Test;
import simulation.EESimulator.EESimulator;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Random;

import static org.junit.Assert.*;

public class CANTest {

	@BeforeClass
	public static void oneTimeSetUp() {
		BusTestUtils.init();
	}

   @Test
   public void testSimulateUntil()  {
       EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
       CAN can = null;
       List<Bus> buses = BusTestUtils.createBusStructure(eeSimulator);
       for (Bus bus: buses) {
           if( bus.getConnectedComponents().size() == 5){
               can = (CAN) bus;
           }
       }
       assertNotNull(can);

       Instant currentTime = Instant.EPOCH;

       BusMessage low = BusTestUtils.createNregisterMessage(can, "low", 2, CAN.MAX_PAYLOAD_SIZE*3, 0);
       long transmissionTimeNs = can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE, low.getTransmittedBytes());


       //stop transmission in payload
       transmissionTimeNs = can.calculateTransmissionTime(CAN.HEADER_SIZE + CAN.MAX_PAYLOAD_SIZE/2);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE + CAN.MAX_PAYLOAD_SIZE/2, low.getTransmittedBytes());

       //add higher prio message
       BusMessage medium = BusTestUtils.createNregisterMessage(can, "medium", 0,CAN.MAX_PAYLOAD_SIZE*2, 1);

       //finish off packet
       transmissionTimeNs = can.calculateTransmissionTime(CAN.MAX_PAYLOAD_SIZE/2 + CAN.TRAILER_SIZE);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());


       //stop transmission in header
       transmissionTimeNs = can.calculateTransmissionTime(CAN.HEADER_SIZE -2);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());
       assertEquals(0, medium.getTransmittedBytes());

       //add higher prio message
       BusMessage high = BusTestUtils.createNregisterMessage(can, "high", 1, CAN.MAX_PAYLOAD_SIZE, 2);

       //finish off packet
       transmissionTimeNs = can.calculateTransmissionTime(2 + CAN.MAX_PAYLOAD_SIZE + CAN.TRAILER_SIZE);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());
       assertEquals(CAN.MAX_PAYLOAD_SIZE, medium.getTransmittedBytes());


       //stop transmission in trailer
       transmissionTimeNs = can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE-3);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());
       assertEquals(CAN.MAX_PAYLOAD_SIZE, medium.getTransmittedBytes());
       assertEquals(CAN.MAX_PAYLOAD_SIZE, high.getTransmittedBytes());

       //finish off packet
       transmissionTimeNs = can.calculateTransmissionTime(3);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());
       assertEquals(CAN.MAX_PAYLOAD_SIZE, medium.getTransmittedBytes());
       assertEquals(can.getCurrentTime(), high.getFinishTime());

       transmissionTimeNs = can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(CAN.MAX_PAYLOAD_SIZE * 2, low.getTransmittedBytes());
       assertEquals(can.getCurrentTime(), medium.getFinishTime());

       transmissionTimeNs = can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE);
       can.simulateUntil(can.getCurrentTime().plusNanos(transmissionTimeNs));
       currentTime = currentTime.plusNanos(transmissionTimeNs);
       assertEquals(currentTime, can.getCurrentTime());
       assertEquals(can.getCurrentTime(), low.getFinishTime());
   }

   @Test
   public void testSimulateUntilRandomized() {
       EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
       CAN can = null;
       List<Bus> buses = BusTestUtils.createBusStructure(eeSimulator);
       for (Bus bus: buses) {
           if(bus.getConnectedComponents().size() ==5){
               can = (CAN) bus;
           }
       }
       assertNotNull(can);

       List<BusMessage> msgs = BusTestUtils.createNregisterMessages(can);
       PriorityQueue<BusMessage> messages = new PriorityQueue<>(new BusMessageComparatorIdDesc());
       messages.addAll(msgs);

       while (!messages.isEmpty()) {
           BusMessage msg = messages.poll();
           assertEquals(0, msg.getTransmittedBytes());

           System.out.println("--------------New Message-------------");
           System.out.println("--------------"+ messages.size() + " remaining msgs-------------");
           int fullPackets = msg.getRemainingBytes()/CAN.MAX_PAYLOAD_SIZE;
           Instant time = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE*fullPackets));
           //finish off remaining bytes
           int remaingBytes = msg.getRemainingBytes() % CAN.MAX_PAYLOAD_SIZE;
           System.out.println("Message size: " + msg.getRemainingBytes() + "; Full packets: " + fullPackets + "; Remaining bytes: " + remaingBytes);
           if(remaingBytes != 0){
               time = time.plusNanos(can.calculateTransmissionTime(CAN.HEADER_SIZE + remaingBytes + CAN.TRAILER_SIZE));
           }
           can.simulateUntil(time);

           assertTrue(msg.isTransmitted());
       }
   }

   @Test
   public void testGetNextFinishTime() {
       EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
       CAN can = null;
       List<Bus> buses = BusTestUtils.createBusStructure(eeSimulator);
       for (Bus bus: buses) {
           if(bus.getConnectedComponents().size() ==5){
               can = (CAN) bus;
           }
       }
       assertNotNull(can);

       BusTestUtils.createNregisterMessage(can, "2.5 frames", 0, Math.toIntExact(Math.round(CAN.MAX_PAYLOAD_SIZE * 2.5)), 0);
       Instant expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE * 2 + (CAN.MAX_PAYLOAD_SIZE/2) + CAN.HEADER_SIZE + CAN.TRAILER_SIZE));
       assertEquals(expected, can.getNextFinishTime());

       BusMessage msg = BusTestUtils.createNregisterMessage(can, "2 frames", 0,CAN.MAX_PAYLOAD_SIZE * 2, 1);
       expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE * 2));
       assertEquals(expected, can.getNextFinishTime());

       can.simulateUntil(can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE)));
       assertEquals(CAN.MAX_PAYLOAD_SIZE, msg.getRemainingBytes());
       expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE));
       assertEquals(expected, can.getNextFinishTime());

       can.simulateUntil(can.getCurrentTime().plusNanos(can.calculateTransmissionTime(10)));
       expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE-10));
       assertEquals(expected, can.getNextFinishTime());

       can.simulateUntil(can.getCurrentTime().plusNanos(can.calculateTransmissionTime(5)));
       expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE - 15));
       assertEquals(expected, can.getNextFinishTime());

       can.simulateUntil(can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE)));
       expected = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE * 2 + CAN.MAX_PAYLOAD_SIZE/2 + CAN.HEADER_SIZE + CAN.TRAILER_SIZE - 15));
       assertEquals(expected, can.getNextFinishTime());
       can.simulateUntil(can.getNextFinishTime());

       assertFalse(can.hasMessages());

       BusMessage msg1 = BusTestUtils.createNregisterMessage(can, 0, 0, 808,3);
       BusMessage msg2 = BusTestUtils.createNregisterMessage(can, 0, 0, 746,2);
       BusMessage msg3 = BusTestUtils.createNregisterMessage(can, 0, 0, 683,1);

       can.simulateUntil(can.getNextFinishTime());
       assertTrue(msg1.isTransmitted());
       assertEquals(0, msg2.getTransmittedBytes());
       can.simulateUntil(can.getNextFinishTime());
       assertTrue(msg2.isTransmitted());
       assertEquals(0, msg3.getTransmittedBytes());
       can.simulateUntil(can.getNextFinishTime());
       assertEquals(msg3.getMessageLen(), msg3.getTransmittedBytes());
       assertTrue(msg3.isTransmitted());
   }

   @Test
   public void testGetNextFinishTimeRandomized() {
       EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
       CAN can = null;
       List<Bus> buses = BusTestUtils.createBusStructure(eeSimulator);
       for (Bus bus: buses) {
           if(bus.getConnectedComponents().size() ==5){
               can = (CAN) bus;
           }
       }
       assertNotNull(can);

       List<BusMessage> msgs = BusTestUtils.createNregisterMessages(can);
       PriorityQueue<BusMessage> messages = new PriorityQueue<>(new BusMessageComparatorIdDesc());
       messages.addAll(msgs);

       while (!messages.isEmpty()) {
           BusMessage msg = messages.poll();
           assertEquals(0, msg.getTransmittedBytes());

           System.out.println("--------------New Message-------------");
           System.out.println("--------------"+ messages.size() + " remaining msgs-------------");
           Instant nextFinishTime = can.getNextFinishTime();
           int fullPackets = msg.getRemainingBytes()/CAN.MAX_PAYLOAD_SIZE;
           Instant time = can.getCurrentTime().plusNanos(can.calculateTransmissionTime(CAN.MAX_MESSAGE_SIZE*fullPackets));
           //finish off remaining bytes
           int remaingBytes = msg.getRemainingBytes() % CAN.MAX_PAYLOAD_SIZE;
           System.out.println("Message size: " + msg.getRemainingBytes() + "; Full packets: " + fullPackets + "; Remaining bytes: " + remaingBytes);
           if(remaingBytes != 0){
               time = time.plusNanos(can.calculateTransmissionTime(CAN.HEADER_SIZE + remaingBytes + CAN.TRAILER_SIZE));
           }
           assertEquals(time, nextFinishTime);
           can.simulateUntil(nextFinishTime);

           assertEquals(msg.getMessageLen(), msg.getTransmittedBytes());
           assertTrue(msg.isTransmitted());
       }
   }


}
