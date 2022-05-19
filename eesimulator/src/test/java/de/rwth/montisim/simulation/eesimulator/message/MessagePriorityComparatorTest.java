/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.message;

import java.time.Instant;
import java.util.Arrays;
import java.util.PriorityQueue;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bus.MessageTransmission;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANMessageTransmission;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class MessagePriorityComparatorTest {
    @Test
    public void testComparator() throws EEMessageTypeException {
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));
        TestEEComponent c1 = new TestEEComponent("comp1", eesystem);
        TestEEComponent c2 = new TestEEComponent("comp2", eesystem);

        eesystem.addComponentPriorities(Arrays.asList(
                new Pair<String, Integer>("comp1", 1),
                new Pair<String, Integer>("comp2", 2)
        ));

        PriorityQueue<CANMessageTransmission> messages = new PriorityQueue<CANMessageTransmission>(
                new MessageTransmission.MsgTransPriorityComp(eesystem.getMsgPrioComp())
        );

        c1.addPort(PortInformation.newOptionalOutputDataPort("msg1", BasicType.DOUBLE));
        c1.addPort(PortInformation.newOptionalOutputDataPort("msg2", BasicType.DOUBLE));
        c1.addPort(PortInformation.newOptionalOutputDataPort("msg3", BasicType.DOUBLE));
        c2.addPort(PortInformation.newOptionalOutputDataPort("msg4", BasicType.DOUBLE));

        Message msg1 = new Message(c1.getMsgInfo("msg1"), null, 1);
        Message msg2 = new Message(c1.getMsgInfo("msg2"), null, 1);
        Message msg3 = new Message(c1.getMsgInfo("msg3"), null, 1);
        Message msg4 = new Message(c2.getMsgInfo("msg4"), null, 1);

        eesystem.addMessagePriorities(Arrays.asList(
                new Pair<String, Integer>("msg1", 1),
                new Pair<String, Integer>("msg2", 2),
                new Pair<String, Integer>("msg3", 3),
                new Pair<String, Integer>("msg4", 1)
        ));


        messages.offer(new CANMessageTransmission(msg2, null));
        Assert.assertEquals(msg2, messages.peek().msg);
        messages.offer(new CANMessageTransmission(msg1, null));
        Assert.assertEquals(msg1, messages.peek().msg);
        messages.offer(new CANMessageTransmission(msg3, null));
        Assert.assertEquals(msg1, messages.peek().msg);
        messages.offer(new CANMessageTransmission(msg4, null));
        Assert.assertEquals(msg4, messages.poll().msg);
        Assert.assertEquals(msg1, messages.poll().msg);
        Assert.assertEquals(msg2, messages.poll().msg);
        Assert.assertEquals(msg3, messages.poll().msg);
    }
}