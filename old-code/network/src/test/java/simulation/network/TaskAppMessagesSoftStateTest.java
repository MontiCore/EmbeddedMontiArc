/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

import de.rwth.montisim.commons.simulation.SimulationLoopExecutable;
import org.junit.Test;
import simulation.network.settings.SettingsSimple;
import simulation.network.tasks.TaskAppMessagesSoftState;
import de.rwth.montisim.simulation.util.Log;

import java.time.Instant;
import java.time.Duration;
import java.util.*;


import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Test for basic message soft state cleanup functionality
 */
public class TaskAppMessagesSoftStateTest {

    @Test
    public void testSoftStateRemain() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        Map<String, List<NetworkMessage>> messagesMap = Collections.synchronizedMap(new HashMap<>());
        List<NetworkMessage> messageList1 = Collections.synchronizedList(new LinkedList<>());
        List<NetworkMessage> messageList2 = Collections.synchronizedList(new LinkedList<>());

        for (int i = 0; i < 10; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList1.add(msg);
        }

        for (int i = 0; i < 5; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList2.add(msg);
        }

        messagesMap.put("fea86fb7a631", messageList1);
        messagesMap.put("fec2efa52351", messageList2);

        // Ensure that keeping valid messages works
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertTrue(messagesMap.size() == 2);
        assertTrue(messagesMap.get("fea86fb7a631").size() == 10);
        assertTrue(messagesMap.get("fec2efa52351").size() == 5);
        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, settings.getMessageBufferMaxTime());
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertTrue(messagesMap.size() == 2);
        assertTrue(messagesMap.get("fea86fb7a631").size() == 10);
        assertTrue(messagesMap.get("fec2efa52351").size() == 5);

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testSoftStateClear() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        Map<String, List<NetworkMessage>> messagesMap = Collections.synchronizedMap(new HashMap<>());
        List<NetworkMessage> messageList1 = Collections.synchronizedList(new LinkedList<>());
        List<NetworkMessage> messageList2 = Collections.synchronizedList(new LinkedList<>());

        for (int i = 0; i < 10; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList1.add(msg);
        }

        for (int i = 0; i < 5; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList2.add(msg);
        }

        messagesMap.put("fea86fb7a631", messageList1);
        messagesMap.put("fec2efa52351", messageList2);

        // Ensure that clearing invalid messages works
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertTrue(messagesMap.size() == 2);
        assertTrue(messagesMap.get("fea86fb7a631").size() == 10);
        assertTrue(messagesMap.get("fec2efa52351").size() == 5);
        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, settings.getMessageBufferMaxTime().multipliedBy(2L).plusMillis(2));
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertEquals(0, messagesMap.size());

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testSoftStateMixed() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        Map<String, List<NetworkMessage>> messagesMap = Collections.synchronizedMap(new HashMap<>());
        List<NetworkMessage> messageList1 = Collections.synchronizedList(new LinkedList<>());
        List<NetworkMessage> messageList2 = Collections.synchronizedList(new LinkedList<>());

        for (int i = 0; i < 10; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList1.add(msg);
        }
        for (int i = 0; i < 5; ++i) {
            long msgReceiveTimeNs = (i * (settings.getMessageBufferMaxTime().toNanos() / 10)) + 1;
            NetworkMessage msg = new NetworkMessage();
            msg.setSimReceiveTime(Instant.ofEpochSecond(0, msgReceiveTimeNs));
            messageList2.add(msg);
        }

        messagesMap.put("fea86fb7a631", messageList1);
        messagesMap.put("fec2efa52351", messageList2);

        // Ensure some messages are kept and others are removed / cleared
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertTrue(messagesMap.size() == 2);
        assertTrue(messagesMap.get("fea86fb7a631").size() == 10);
        assertTrue(messagesMap.get("fec2efa52351").size() == 5);
        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, settings.getMessageBufferMaxTime().dividedBy(2L).plus(settings.getMessageBufferMaxTime()));
        TaskAppMessagesSoftState.softStateCleanup(messagesMap);
        assertEquals(1, messagesMap.size());
        assertEquals(5, messagesMap.get("fea86fb7a631").size());

        // Enable log
        Log.setLogEnabled(true);
    }
}
