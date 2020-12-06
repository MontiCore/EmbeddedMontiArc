/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.bus.can;

import java.time.Duration;
import java.time.Instant;
import java.util.Arrays;

import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.Time;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.events.MessageSendEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class CANTest {
    Instant startTime;
    MessageTypeManager mtManager;
    EESystem eesystem;
    CAN can;
    TestEEComponent c1, c2, c3;
    Message m1, m2, m3, m4, m5;
    CANMessageTransmission t1, t2, t3, t4, t5;

    /**
     * All tests assume HIGH_SPEED_CAN transmission speed. The setup uses 3
     * TestEEComponents connected to a CAN bus. Routing: every messages is sent to
     * all 3 test components.
     * 
     * For details: [docs/eesimulator.md]
     * 
     * @throws EEMessageTypeException
     */
    @Before
    public void setup() throws EEMessageTypeException {
        //startTime = Instant.now();
        startTime = Instant.EPOCH;
        mtManager = new MessageTypeManager();
        eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH), mtManager);
        can = new CAN(new CANProperties().setBitRate(CAN.HIGH_SPEED_CAN_BITRATE).setName("TestCanBus"), eesystem.getMsgPrioComp());
        can.attachTo(eesystem);
        c1 = new TestEEComponent("TestComponent1"); c1.attachTo(eesystem);
        c2 = new TestEEComponent("TestComponent2"); c2.attachTo(eesystem);
        c3 = new TestEEComponent("TestComponent3"); c3.attachTo(eesystem);
        // Not necessary
        can.addComponent(c1);
        can.addComponent(c2);
        can.addComponent(c3);

        // Full frame payload
        m1 = new Message(null, mtManager.registerMessage("m1", BasicType.DOUBLE, c1), null, CAN.MAX_PAYLOAD_SIZE_BYTES);
        t1 = new CANMessageTransmission(m1, null);
        // Partial frame payload
        m2 = new Message(null, mtManager.registerMessage("m2", BasicType.DOUBLE, c1), null, 3);
        t2 = new CANMessageTransmission(m2, null);
        // Multi frame payload (full)
        m3 = new Message(null, mtManager.registerMessage("m3", BasicType.DOUBLE, c1), null, CAN.MAX_PAYLOAD_SIZE_BYTES*5);
        t3 = new CANMessageTransmission(m3, null);
        // Multi frame payload (full)
        m4 = new Message(null, mtManager.registerMessage("m4", BasicType.DOUBLE, c1), null, CAN.MAX_PAYLOAD_SIZE_BYTES*3 + 5);
        t4 = new CANMessageTransmission(m4, null);
        // Full frame payload
        m5 = new Message(null, mtManager.registerMessage("m5", BasicType.DOUBLE, c1), null, CAN.MAX_PAYLOAD_SIZE_BYTES);
        t5 = new CANMessageTransmission(m5, null);
        
        eesystem.getMessageTypeManager().addMessagePriorities(Arrays.asList(
            new Pair<String, Integer>("m1", 1),
            new Pair<String, Integer>("m2", 2),
            new Pair<String, Integer>("m3", 3),
            new Pair<String, Integer>("m4", 4),
            new Pair<String, Integer>("m5", 5)
        ));

        // Register messages routing.
        can.addMessageTargets(m1.msgInfo, Arrays.asList(c1,c2,c3));
        can.addMessageTargets(m2.msgInfo, Arrays.asList(c1,c2,c3));
        can.addMessageTargets(m3.msgInfo, Arrays.asList(c1,c2,c3));
        can.addMessageTargets(m4.msgInfo, Arrays.asList(c1,c2,c3));
        can.addMessageTargets(m5.msgInfo, Arrays.asList(c1,c2,c3));
    }

    @Test
    public void fullFrameMsg(){
        // Create events
        MessageSendEvent m1send = new MessageSendEvent(can, startTime, m1);
        eesystem.simulator.addEvent(m1send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(1)));

        // Verify (Test all components once)
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c1.", 1, c1.events.size());
        validateReceiveEvent(m1send, c1.events.get(0), bitsToNanos(t1.requiredTotalBits));
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c2.", 1, c2.events.size());
        validateReceiveEvent(m1send, c2.events.get(0), bitsToNanos(t1.requiredTotalBits));
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c3.", 1, c3.events.size());
        validateReceiveEvent(m1send, c3.events.get(0), bitsToNanos(t1.requiredTotalBits));
    }

    @Test
    public void partialFrameMsg(){
        // Create events
        MessageSendEvent m2send = new MessageSendEvent(can, startTime.plus(Duration.ofMillis(3)), m2);
        eesystem.simulator.addEvent(m2send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c1.", 1, c1.events.size());
        validateReceiveEvent(m2send, c1.events.get(0), bitsToNanos(t2.requiredTotalBits));
    }

    @Test
    public void multiFullFrameMsg(){
        // Create events
        MessageSendEvent m3send = new MessageSendEvent(can, startTime.plus(Duration.ofSeconds(1)), m3);
        eesystem.simulator.addEvent(m3send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c1.", 1, c1.events.size());
        validateReceiveEvent(m3send, c1.events.get(0), bitsToNanos(t3.requiredTotalBits));
    }

    @Test
    public void multiPartialFrameMsg(){
        // Create events
        MessageSendEvent m4send = new MessageSendEvent(can, startTime.plus(Duration.ofSeconds(1)), m4);
        eesystem.simulator.addEvent(m4send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent c1.", 1, c1.events.size());
        validateReceiveEvent(m4send, c1.events.get(0), bitsToNanos(t4.requiredTotalBits));
    }

    @Test
    public void multiMsgsInTime(){
        // Create events
        MessageSendEvent m1send = new MessageSendEvent(can, startTime.plus(Duration.ofMillis(3)), m1);
        MessageSendEvent m2send = new MessageSendEvent(can, startTime.plus(Duration.ofMillis(15)), m2);
        MessageSendEvent m3send = new MessageSendEvent(can, startTime.plus(Duration.ofMillis(200)), m3);
        MessageSendEvent m4send = new MessageSendEvent(can, startTime.plus(Duration.ofMillis(700)), m4);

        eesystem.simulator.addEvent(m1send);
        eesystem.simulator.addEvent(m2send);
        eesystem.simulator.addEvent(m3send);
        eesystem.simulator.addEvent(m4send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent.", 4, c1.events.size());
        validateReceiveEvent(m1send, c1.events.get(0), bitsToNanos(t1.requiredTotalBits));
        validateReceiveEvent(m2send, c1.events.get(1), bitsToNanos(t2.requiredTotalBits));
        validateReceiveEvent(m3send, c1.events.get(2), bitsToNanos(t3.requiredTotalBits));
        validateReceiveEvent(m4send, c1.events.get(3), bitsToNanos(t4.requiredTotalBits));
    }
    @Test
    public void multiMsgsAtSameTime(){
        // Create events
        MessageSendEvent m1send = new MessageSendEvent(can, startTime, m1);
        MessageSendEvent m2send = new MessageSendEvent(can, startTime, m2);
        MessageSendEvent m3send = new MessageSendEvent(can, startTime, m3);
        MessageSendEvent m4send = new MessageSendEvent(can, startTime, m4);

        // Manual process for order
        can.process(m2send);
        can.process(m4send);
        can.process(m1send);
        can.process(m3send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify (Priorities should make the message arrive in order)
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent.", 4, c1.events.size());
        long t = t1.requiredTotalBits;
        validateReceiveEvent(m1send, c1.events.get(0), bitsToNanos(t));
        t += CAN.INTERFRAME_SPACE_BITS + t2.requiredTotalBits;
        validateReceiveEvent(m2send, c1.events.get(1), bitsToNanos(t));
        t += CAN.INTERFRAME_SPACE_BITS + t3.requiredTotalBits;
        validateReceiveEvent(m3send, c1.events.get(2), bitsToNanos(t));
        t += CAN.INTERFRAME_SPACE_BITS + t4.requiredTotalBits;
        validateReceiveEvent(m4send, c1.events.get(3), bitsToNanos(t));
    }

    @Test
    public void multiMsgsOverlappingTime(){
        // Create events
        MessageSendEvent m4send = new MessageSendEvent(can, startTime, m4);
        MessageSendEvent m3send = new MessageSendEvent(can, startTime.plus(Duration.ofNanos(bitsToNanos(CAN.FULL_FRAME))), m3);
        MessageSendEvent m1send = new MessageSendEvent(can, startTime.plus(Duration.ofNanos(bitsToNanos(CAN.FULL_FRAME*3 - 20))), m1);
        MessageSendEvent m1send2 = new MessageSendEvent(can, startTime.plus(Duration.ofNanos(bitsToNanos(CAN.FULL_FRAME*7))), m1);
        MessageSendEvent m5send = new MessageSendEvent(can, startTime.plus(Duration.ofNanos(bitsToNanos(CAN.FULL_FRAME*10-100))), m5);
        
        eesystem.simulator.addEvent(m4send);
        eesystem.simulator.addEvent(m3send);
        eesystem.simulator.addEvent(m1send);
        eesystem.simulator.addEvent(m1send2);
        eesystem.simulator.addEvent(m5send);

        // Perform computation through EESimulator
        eesystem.simulator.update(new TimeUpdate(startTime, Duration.ofSeconds(2)));

        // Verify (Priorities should make the message arrive in order)
        Assert.assertEquals("Unexpected number of events arrived at the TestEEComponent.", 5, c1.events.size());
        validateReceiveEvent(m1send, c1.events.get(0), bitsToNanos(CAN.FULL_FRAME-CAN.INTERFRAME_SPACE_BITS+20));
        validateReceiveEvent(m3send, c1.events.get(1), bitsToNanos(CAN.FULL_FRAME*6-CAN.INTERFRAME_SPACE_BITS));
        validateReceiveEvent(m1send2, c1.events.get(2), bitsToNanos(CAN.FULL_FRAME-CAN.INTERFRAME_SPACE_BITS));
        long b4 = t4.requiredTotalBits + (t3.requiredTotalBits + t1.requiredTotalBits*2 + CAN.INTERFRAME_SPACE_BITS*3);
        validateReceiveEvent(m4send, c1.events.get(3), bitsToNanos(b4));
        long b5 = 100 + t5.requiredTotalBits + (t4.requiredTotalBits-(t4.requiredFrames-1)*CAN.FULL_FRAME + CAN.INTERFRAME_SPACE_BITS);
        validateReceiveEvent(m5send, c1.events.get(4), bitsToNanos(b5));
    }

    public void validateReceiveEvent(MessageSendEvent sendEvent, MessageReceiveEvent evt, long expectedTransmissionDurationNanos){
        Assert.assertEquals("Wrong message.", sendEvent.getMessage(), evt.getMessage());
        Instant expectedArrival = sendEvent.getEventTime().plus(Duration.ofNanos(expectedTransmissionDurationNanos));
        Assert.assertEquals("Wrong message arrival time.", expectedArrival, evt.getEventTime());
    }

    public long bitsToNanos(long bits){
        return (Time.SECOND_TO_NANOSEC*bits)/CAN.HIGH_SPEED_CAN_BITRATE;
    }
}