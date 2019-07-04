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
package simulation.network;

import commons.simulation.SimulationLoopExecutable;
import org.junit.Test;
import simulation.network.settings.SettingsSimple;
import simulation.network.tasks.TaskAppMessagesSoftState;
import simulation.util.Log;

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