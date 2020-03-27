/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class ComponentSetupTest {

    @Test
    public void singleBus() throws EESetupException {
        // Create an EE Setup.
        EESimulator simulator = new EESimulator();
        Bus bus = ConstantBus.newInstantBus(simulator, "TestInstantBus");
        TestEEComponent c1, c2, c3;
        c1 = new TestEEComponent(simulator, "TestComponent1");
        c2 = new TestEEComponent(simulator, "TestComponent2");
        c3 = new TestEEComponent(simulator, "TestComponent3");

        MessageInformation m1 = c1.addInput("m1", DataType.newIntType());
        MessageInformation m2 = c1.addInput("m2", DataType.newDoubleType());
        
        c2.addInput("m1", DataType.newIntType());
        MessageInformation m3 = c2.addInput("m3", DataType.newIntType());
        MessageInformation m4 = c2.addOutput("m4", DataType.newDoubleType());
        MessageInformation m5 = c2.addOutput("m5", DataType.newDoubleType());

        c3.addInput("m4", DataType.newDoubleType());
        c3.addOutput("m1", DataType.newIntType());
        c3.addOutput("m2", DataType.newDoubleType());
        c3.addOutput("m3", DataType.newIntType());

        // Test different connection methods
        c1.connectToBus(bus);
        c2.connectToBus("TestInstantBus");
        c3.connectToBus(bus.id);

        simulator.finalizeSetup();

        // Validate Routing
        HashMap<Integer, HashSet<Bus>> c1MsgTargets = new HashMap<>();
        assertContent(c1.msgTargets, c1MsgTargets);
        
        HashSet<Bus> busTarget = new HashSet<>();
        busTarget.add(bus);

        HashMap<Integer, HashSet<Bus>> c2MsgTargets = new HashMap<>();
        c2MsgTargets.put(m4.messageId, busTarget);
        c2MsgTargets.put(m5.messageId, new HashSet<>());
        assertContent(c2.msgTargets, c2MsgTargets);

        HashMap<Integer, HashSet<Bus>> c3MsgTargets = new HashMap<>();
        c3MsgTargets.put(m1.messageId, busTarget);
        c3MsgTargets.put(m2.messageId, busTarget);
        c3MsgTargets.put(m3.messageId, busTarget);
        assertContent(c3.msgTargets, c3MsgTargets);

        HashMap<Integer, HashSet<BusComponent>> busTargets = new HashMap<>();

        HashSet<BusComponent> m1Targets = new HashSet<>();
        m1Targets.add(c1);
        m1Targets.add(c2);
        busTargets.put(m1.messageId, m1Targets);
        
        HashSet<BusComponent> m2Targets = new HashSet<>();
        m2Targets.add(c1);
        busTargets.put(m2.messageId, m2Targets);

        HashSet<BusComponent> m3Targets = new HashSet<>();
        m3Targets.add(c2);
        busTargets.put(m3.messageId, m3Targets);

        HashSet<BusComponent> m4Targets = new HashSet<>();
        m4Targets.add(c3);
        busTargets.put(m4.messageId, m4Targets);

        assertContent(bus.getMsgTargets(), busTargets);

        //TODO Test message transmission?
    }

    @Test
    public void multiBus() throws EESetupException {
        EESimulator simulator = new EESimulator();

        // Create an EE Setup.
        Bus b1 = ConstantBus.newInstantBus(simulator, "b1");
        Bus b2 = ConstantBus.newInstantBus(simulator, "b2");
        Bus b3 = ConstantBus.newInstantBus(simulator, "b3");
        Bus b4 = ConstantBus.newInstantBus(simulator, "b4");
        Bus b5 = ConstantBus.newInstantBus(simulator, "b5");
        TestEEComponent c1, c2, c3, c4, c5, c6;
        c1 = new TestEEComponent(simulator, "c1");
        c2 = new TestEEComponent(simulator, "c2");
        c3 = new TestEEComponent(simulator, "c3");
        c4 = new TestEEComponent(simulator, "c4");
        c5 = new TestEEComponent(simulator, "c5");
        c6 = new TestEEComponent(simulator, "c6");
        Bridge r1 = Bridge.newInstantBridge(simulator, "r1");
        Bridge r2 = Bridge.newInstantBridge(simulator, "r2");

        // Set ports
        MessageInformation m1 = c1.addOutput("m1", DataType.newIntType());
        c2.addInput("m1", DataType.newIntType());
        c3.addInput("m1", DataType.newIntType());
        c4.addInput("m1", DataType.newIntType());
        c5.addInput("m1", DataType.newIntType());
        c6.addInput("m1", DataType.newIntType());

        MessageInformation m2 = c1.addOutput("m2", DataType.newDoubleType());
        c2.addInput("m2", DataType.newDoubleType());

        MessageInformation m3 = c1.addOutput("m3", DataType.newIntType());
        c3.addInput("m3", DataType.newIntType());

        MessageInformation m4 = c1.addOutput("m4", DataType.newDoubleType());
        c4.addInput("m4", DataType.newDoubleType());

        MessageInformation m5 = c1.addOutput("m5", DataType.newIntType());
        c4.addInput("m5", DataType.newIntType());
        c5.addInput("m5", DataType.newIntType());

        MessageInformation m6 = c1.addOutput("m6", DataType.newDoubleType());

        MessageInformation m7 = c1.addOutput("m7", DataType.newIntType());
        c3.addInput("m7", DataType.newIntType());
        c6.addInput("m7", DataType.newIntType());
        
        // Connect components and bridges
        c1.connectToBus(b1);
        c1.connectToBus(b5);
        c2.connectToBus(b1);
        c3.connectToBus(b2);
        c4.connectToBus(b4);
        c5.connectToBus(b3);
        c6.connectToBus(b5);

        r1.connectToBus(b1);
        r1.connectToBus(b2);
        r2.connectToBus(b2);
        r2.connectToBus(b3);
        r2.connectToBus(b4);

        // Create "Parallel" Setup

        Bus bA = ConstantBus.newInstantBus(simulator, "bA");
        TestEEComponent cA, cB;
        cA = new TestEEComponent(simulator, "cA");
        cB = new TestEEComponent(simulator, "cB");

        // Set ports
        MessageInformation mA = cA.addOutput("mA", DataType.newIntType());
        cB.addInput("mA", DataType.newIntType());

        MessageInformation mB = cB.addOutput("mB", DataType.newDoubleType());
        cA.addInput("mB", DataType.newDoubleType());

        // Connect components and bridges
        cA.connectToBus(bA);
        cB.connectToBus(bA);

        simulator.finalizeSetup();

        // Validate Routing of main setup
        // Components

        HashSet<Bus> targetB1 = new HashSet<>();
        targetB1.add(b1);
        HashSet<Bus> targetB2 = new HashSet<>();
        targetB2.add(b2);
        HashSet<Bus> targetB4 = new HashSet<>();
        targetB4.add(b4);
        HashSet<Bus> targetB1B5 = new HashSet<>();
        targetB1B5.add(b1);
        targetB1B5.add(b5);
        HashSet<Bus> targetB3B4 = new HashSet<>();
        targetB3B4.add(b3);
        targetB3B4.add(b4);

        // C1
        HashMap<Integer, HashSet<Bus>> c1MsgTargets = new HashMap<>();
        c1MsgTargets.put(m1.messageId, targetB1B5);
        c1MsgTargets.put(m2.messageId, targetB1);
        c1MsgTargets.put(m3.messageId, targetB1);
        c1MsgTargets.put(m4.messageId, targetB1);
        c1MsgTargets.put(m5.messageId, targetB1);
        c1MsgTargets.put(m6.messageId, new HashSet<>());
        c1MsgTargets.put(m7.messageId, targetB1B5);
        assertContent(c1.msgTargets, c1MsgTargets);
        
        // C2, C3, C4, C5, C6
        assertContent(c2.msgTargets, new HashMap<>());
        assertContent(c3.msgTargets, new HashMap<>());
        assertContent(c4.msgTargets, new HashMap<>());
        assertContent(c5.msgTargets, new HashMap<>());
        assertContent(c6.msgTargets, new HashMap<>());

        // Bridges
        HashMap<Integer, HashSet<Bus>> r1MsgTargets = new HashMap<>();
        r1MsgTargets.put(m1.messageId, targetB2);
        r1MsgTargets.put(m3.messageId, targetB2);
        r1MsgTargets.put(m4.messageId, targetB2);
        r1MsgTargets.put(m5.messageId, targetB2);
        r1MsgTargets.put(m7.messageId, targetB2);
        assertContent(r1.msgTargets, r1MsgTargets);

        HashMap<Integer, HashSet<Bus>> r2MsgTargets = new HashMap<>();
        r2MsgTargets.put(m1.messageId, targetB3B4);
        r2MsgTargets.put(m4.messageId, targetB4);
        r2MsgTargets.put(m5.messageId, targetB3B4);
        assertContent(r2.msgTargets, r2MsgTargets);

        // Buses

        HashSet<BusComponent> targetC2 = new HashSet<>();
        targetC2.add(c2);
        HashSet<BusComponent> targetC3 = new HashSet<>();
        targetC3.add(c3);
        HashSet<BusComponent> targetC4 = new HashSet<>();
        targetC4.add(c4);
        HashSet<BusComponent> targetC5 = new HashSet<>();
        targetC5.add(c5);
        HashSet<BusComponent> targetC6 = new HashSet<>();
        targetC6.add(c6);
        HashSet<BusComponent> targetR1 = new HashSet<>();
        targetR1.add(r1);
        HashSet<BusComponent> targetR2 = new HashSet<>();
        targetR2.add(r2);
        HashSet<BusComponent> targetR1C2 = new HashSet<>();
        targetR1C2.add(r1);
        targetR1C2.add(c2);
        HashSet<BusComponent> targetR2C3 = new HashSet<>();
        targetR2C3.add(r2);
        targetR2C3.add(c3);

        HashMap<Integer, HashSet<BusComponent>> b1Targets = new HashMap<>();
        b1Targets.put(m1.messageId, targetR1C2);
        b1Targets.put(m2.messageId, targetC2);
        b1Targets.put(m3.messageId, targetR1);
        b1Targets.put(m4.messageId, targetR1);
        b1Targets.put(m5.messageId, targetR1);
        b1Targets.put(m7.messageId, targetR1);
        assertContent(b1.getMsgTargets(), b1Targets);

        HashMap<Integer, HashSet<BusComponent>> b2Targets = new HashMap<>();
        b2Targets.put(m1.messageId, targetR2C3);
        b2Targets.put(m3.messageId, targetC3);
        b2Targets.put(m4.messageId, targetR2);
        b2Targets.put(m5.messageId, targetR2);
        b2Targets.put(m7.messageId, targetC3);
        assertContent(b2.getMsgTargets(), b2Targets);

        HashMap<Integer, HashSet<BusComponent>> b3Targets = new HashMap<>();
        b3Targets.put(m1.messageId, targetC5);
        b3Targets.put(m5.messageId, targetC5);
        assertContent(b3.getMsgTargets(), b3Targets);

        HashMap<Integer, HashSet<BusComponent>> b4Targets = new HashMap<>();
        b4Targets.put(m1.messageId, targetC4);
        b4Targets.put(m4.messageId, targetC4);
        b4Targets.put(m5.messageId, targetC4);
        assertContent(b4.getMsgTargets(), b4Targets);

        HashMap<Integer, HashSet<BusComponent>> b5Targets = new HashMap<>();
        b5Targets.put(m1.messageId, targetC6);
        b5Targets.put(m7.messageId, targetC6);
        assertContent(b5.getMsgTargets(), b5Targets);

        // Validate Routing of Parallel Setup
        
        // Components
        HashSet<Bus> targetBA = new HashSet<>();
        targetBA.add(bA);

        HashMap<Integer, HashSet<Bus>> cAMsgTargets = new HashMap<>();
        cAMsgTargets.put(mA.messageId, targetBA);
        assertContent(cA.msgTargets, cAMsgTargets);

        HashMap<Integer, HashSet<Bus>> cBMsgTargets = new HashMap<>();
        cBMsgTargets.put(mB.messageId, targetBA);
        assertContent(cB.msgTargets, cBMsgTargets);
        
        // Bus

        HashSet<BusComponent> targetCA = new HashSet<>();
        targetCA.add(cA);
        HashSet<BusComponent> targetCB = new HashSet<>();
        targetCB.add(cB);

        HashMap<Integer, HashSet<BusComponent>> bATargets = new HashMap<>();
        bATargets.put(mA.messageId, targetCB);
        bATargets.put(mB.messageId, targetCA);
        assertContent(bA.getMsgTargets(), bATargets);


        //TODO Test message transmission?
    }

    private <T> void assertContent(HashMap<Integer, List<T>> map, HashMap<Integer, HashSet<T>> reference){
        Assert.assertEquals(reference.size(), map.size());
        for (HashMap.Entry<Integer, List<T>> e : map.entrySet()){
            HashSet<T> s = reference.get(e.getKey());
            List<T> l = e.getValue();
            if (s == null && l == null) continue;
            Assert.assertNotNull(s);
            Assert.assertNotNull(l);
            Assert.assertEquals(s.size(), l.size());
            for (T t : l){
                Assert.assertTrue("Object of list not in reference.", s.contains(t));
            }
        }
    }
}