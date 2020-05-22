/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator.message;

import java.util.Arrays;
import java.util.PriorityQueue;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.simulation.eesimulator.EESimulator;
import de.rwth.montisim.simulation.eesimulator.bus.MessageTransmission;
import de.rwth.montisim.simulation.eesimulator.bus.can.CANMessageTransmission;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class MessagePriorityComparatorTest {
    @Test
    public void testComparator() {
        EESimulator sim = new EESimulator();
        new TestEEComponent(sim, "comp1");
        new TestEEComponent(sim, "comp2");

        sim.getComponentManager().addComponentPriorities(Arrays.asList(
            new Pair<String, Integer>("comp1", 1),
            new Pair<String, Integer>("comp2", 2)
        ));

        MessageTypeManager manager = sim.getMessageTypeManager();
        PriorityQueue<CANMessageTransmission> messages = new PriorityQueue<CANMessageTransmission>(
            new MessageTransmission.MsgTransPriorityComp(manager.msgPrioComp)
        );
        Message msg1 = new Message(new MessageInformation("msg1", DataType.DOUBLE, manager, null), null, 1, 0);
        Message msg2 = new Message(new MessageInformation("msg2", DataType.DOUBLE, manager, null), null, 1, 0);
        Message msg3 = new Message(new MessageInformation("msg3", DataType.DOUBLE, manager, null), null, 1, 0);
        Message msg4 = new Message(new MessageInformation("msg4", DataType.DOUBLE, manager, null), null, 1, 1);

        manager.addMessagePriorities(Arrays.asList(
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