/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class ComponentSetupTest {

    @Test
    public void singleBus() throws EESetupException, EEMessageTypeException {
        // Create an EE Setup.
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);
        Bus bus = new ConstantBus(ConstantBusProperties.instantBus().setName("TestInstantBus"));
        bus.attachTo(eesystem);
        TestEEComponent c1, c2, c3;
        c1 = new TestEEComponent("TestComponent1"); c1.attachTo(eesystem);
        c2 = new TestEEComponent("TestComponent2"); c2.attachTo(eesystem);
        c3 = new TestEEComponent("TestComponent3"); c3.attachTo(eesystem);

        MessageInformation m1 = c1.addInput("m1", BasicType.INT);
        MessageInformation m2 = c1.addInput("m2", BasicType.DOUBLE);
        
        c2.addInput("m1", BasicType.INT);
        MessageInformation m3 = c2.addInput("m3", BasicType.INT);
        MessageInformation m4 = c2.addOutput("m4", BasicType.DOUBLE);
        MessageInformation m5 = c2.addOutput("m5", BasicType.DOUBLE);

        c3.addInput("m4", BasicType.DOUBLE);
        c3.addOutput("m1", BasicType.INT);
        c3.addOutput("m2", BasicType.DOUBLE);
        c3.addOutput("m3", BasicType.INT);

        // Test different connection methods
        c1.connectToBus(bus);
        c2.connectToBus("TestInstantBus");
        c3.connectToBus(bus.id);

        eesystem.finalizeSetup();

        // Validate Routing
        HashMap<MessageInformation, HashSet<Bus>> c1MsgTargets = new HashMap<>();
        assertContent(c1.msgTargets, c1MsgTargets);
        
        HashSet<Bus> busTarget = new HashSet<>();
        busTarget.add(bus);

        HashMap<MessageInformation, HashSet<Bus>> c2MsgTargets = new HashMap<>();
        c2MsgTargets.put(m4, busTarget);
        c2MsgTargets.put(m5, new HashSet<>());
        assertContent(c2.msgTargets, c2MsgTargets);

        HashMap<MessageInformation, HashSet<Bus>> c3MsgTargets = new HashMap<>();
        c3MsgTargets.put(m1, busTarget);
        c3MsgTargets.put(m2, busTarget);
        c3MsgTargets.put(m3, busTarget);
        assertContent(c3.msgTargets, c3MsgTargets);

        HashMap<MessageInformation, HashSet<BusUser>> busTargets = new HashMap<>();

        HashSet<BusUser> m1Targets = new HashSet<>();
        m1Targets.add(c1);
        m1Targets.add(c2);
        busTargets.put(m1, m1Targets);
        
        HashSet<BusUser> m2Targets = new HashSet<>();
        m2Targets.add(c1);
        busTargets.put(m2, m2Targets);

        HashSet<BusUser> m3Targets = new HashSet<>();
        m3Targets.add(c2);
        busTargets.put(m3, m3Targets);

        HashSet<BusUser> m4Targets = new HashSet<>();
        m4Targets.add(c3);
        busTargets.put(m4, m4Targets);

        assertContent(bus.getMsgTargets(), busTargets);

        //TODO Test message transmission?
    }

    @Test
    public void multiBus() throws EESetupException, EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);

        // Create an EE Setup.
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"));
        b2.attachTo(eesystem);
        Bus b3 = new ConstantBus(ConstantBusProperties.instantBus().setName("b3"));
        b3.attachTo(eesystem);
        Bus b4 = new ConstantBus(ConstantBusProperties.instantBus().setName("b4"));
        b4.attachTo(eesystem);
        Bus b5 = new ConstantBus(ConstantBusProperties.instantBus().setName("b5"));
        b5.attachTo(eesystem);
        TestEEComponent c1, c2, c3, c4, c5, c6;
        c1 = new TestEEComponent("c1"); c1.attachTo(eesystem);
        c2 = new TestEEComponent("c2"); c2.attachTo(eesystem);
        c3 = new TestEEComponent("c3"); c3.attachTo(eesystem);
        c4 = new TestEEComponent("c4"); c4.attachTo(eesystem);
        c5 = new TestEEComponent("c5"); c5.attachTo(eesystem);
        c6 = new TestEEComponent("c6"); c6.attachTo(eesystem);
        Bridge r1 = new Bridge(BridgeProperties.instantBridge().setName("r1"));
        r1.attachTo(eesystem);
        Bridge r2 = new Bridge(BridgeProperties.instantBridge().setName("r2"));
        r2.attachTo(eesystem);

        // Set ports
        MessageInformation m1 = c1.addOutput("m1", BasicType.INT);
        c2.addInput("m1", BasicType.INT);
        c3.addInput("m1", BasicType.INT);
        c4.addInput("m1", BasicType.INT);
        c5.addInput("m1", BasicType.INT);
        c6.addInput("m1", BasicType.INT);

        MessageInformation m2 = c1.addOutput("m2", BasicType.DOUBLE);
        c2.addInput("m2", BasicType.DOUBLE);

        MessageInformation m3 = c1.addOutput("m3", BasicType.INT);
        c3.addInput("m3", BasicType.INT);

        MessageInformation m4 = c1.addOutput("m4", BasicType.DOUBLE);
        c4.addInput("m4", BasicType.DOUBLE);

        MessageInformation m5 = c1.addOutput("m5", BasicType.INT);
        c4.addInput("m5", BasicType.INT);
        c5.addInput("m5", BasicType.INT);

        MessageInformation m6 = c1.addOutput("m6", BasicType.DOUBLE);

        MessageInformation m7 = c1.addOutput("m7", BasicType.INT);
        c3.addInput("m7", BasicType.INT);
        c6.addInput("m7", BasicType.INT);
        
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

        Bus bA = new ConstantBus(ConstantBusProperties.instantBus().setName("bA"));
        bA.attachTo(eesystem);
        TestEEComponent cA, cB;
        cA = new TestEEComponent("cA"); cA.attachTo(eesystem);
        cB = new TestEEComponent("cB"); cB.attachTo(eesystem);

        // Set ports
        MessageInformation mA = cA.addOutput("mA", BasicType.INT);
        cB.addInput("mA", BasicType.INT);

        MessageInformation mB = cB.addOutput("mB", BasicType.DOUBLE);
        cA.addInput("mB", BasicType.DOUBLE);

        // Connect components and bridges
        cA.connectToBus(bA);
        cB.connectToBus(bA);

        eesystem.finalizeSetup();

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
        HashMap<MessageInformation, HashSet<Bus>> c1MsgTargets = new HashMap<>();
        c1MsgTargets.put(m1, targetB1B5);
        c1MsgTargets.put(m2, targetB1);
        c1MsgTargets.put(m3, targetB1);
        c1MsgTargets.put(m4, targetB1);
        c1MsgTargets.put(m5, targetB1);
        c1MsgTargets.put(m6, new HashSet<>());
        c1MsgTargets.put(m7, targetB1B5);
        assertContent(c1.msgTargets, c1MsgTargets);
        
        // C2, C3, C4, C5, C6
        assertContent(c2.msgTargets, new HashMap<>());
        assertContent(c3.msgTargets, new HashMap<>());
        assertContent(c4.msgTargets, new HashMap<>());
        assertContent(c5.msgTargets, new HashMap<>());
        assertContent(c6.msgTargets, new HashMap<>());

        // Bridges
        HashMap<MessageInformation, HashSet<Bus>> r1MsgTargets = new HashMap<>();
        r1MsgTargets.put(m1, targetB2);
        r1MsgTargets.put(m3, targetB2);
        r1MsgTargets.put(m4, targetB2);
        r1MsgTargets.put(m5, targetB2);
        r1MsgTargets.put(m7, targetB2);
        assertContent(r1.msgTargets, r1MsgTargets);

        HashMap<MessageInformation, HashSet<Bus>> r2MsgTargets = new HashMap<>();
        r2MsgTargets.put(m1, targetB3B4);
        r2MsgTargets.put(m4, targetB4);
        r2MsgTargets.put(m5, targetB3B4);
        assertContent(r2.msgTargets, r2MsgTargets);

        // Buses

        HashSet<BusUser> targetC2 = new HashSet<>();
        targetC2.add(c2);
        HashSet<BusUser> targetC3 = new HashSet<>();
        targetC3.add(c3);
        HashSet<BusUser> targetC4 = new HashSet<>();
        targetC4.add(c4);
        HashSet<BusUser> targetC5 = new HashSet<>();
        targetC5.add(c5);
        HashSet<BusUser> targetC6 = new HashSet<>();
        targetC6.add(c6);
        HashSet<BusUser> targetR1 = new HashSet<>();
        targetR1.add(r1);
        HashSet<BusUser> targetR2 = new HashSet<>();
        targetR2.add(r2);
        HashSet<BusUser> targetR1C2 = new HashSet<>();
        targetR1C2.add(r1);
        targetR1C2.add(c2);
        HashSet<BusUser> targetR2C3 = new HashSet<>();
        targetR2C3.add(r2);
        targetR2C3.add(c3);

        HashMap<MessageInformation, HashSet<BusUser>> b1Targets = new HashMap<>();
        b1Targets.put(m1, targetR1C2);
        b1Targets.put(m2, targetC2);
        b1Targets.put(m3, targetR1);
        b1Targets.put(m4, targetR1);
        b1Targets.put(m5, targetR1);
        b1Targets.put(m7, targetR1);
        assertContent(b1.getMsgTargets(), b1Targets);

        HashMap<MessageInformation, HashSet<BusUser>> b2Targets = new HashMap<>();
        b2Targets.put(m1, targetR2C3);
        b2Targets.put(m3, targetC3);
        b2Targets.put(m4, targetR2);
        b2Targets.put(m5, targetR2);
        b2Targets.put(m7, targetC3);
        assertContent(b2.getMsgTargets(), b2Targets);

        HashMap<MessageInformation, HashSet<BusUser>> b3Targets = new HashMap<>();
        b3Targets.put(m1, targetC5);
        b3Targets.put(m5, targetC5);
        assertContent(b3.getMsgTargets(), b3Targets);

        HashMap<MessageInformation, HashSet<BusUser>> b4Targets = new HashMap<>();
        b4Targets.put(m1, targetC4);
        b4Targets.put(m4, targetC4);
        b4Targets.put(m5, targetC4);
        assertContent(b4.getMsgTargets(), b4Targets);

        HashMap<MessageInformation, HashSet<BusUser>> b5Targets = new HashMap<>();
        b5Targets.put(m1, targetC6);
        b5Targets.put(m7, targetC6);
        assertContent(b5.getMsgTargets(), b5Targets);

        // Validate Routing of Parallel Setup
        
        // Components
        HashSet<Bus> targetBA = new HashSet<>();
        targetBA.add(bA);

        HashMap<MessageInformation, HashSet<Bus>> cAMsgTargets = new HashMap<>();
        cAMsgTargets.put(mA, targetBA);
        assertContent(cA.msgTargets, cAMsgTargets);

        HashMap<MessageInformation, HashSet<Bus>> cBMsgTargets = new HashMap<>();
        cBMsgTargets.put(mB, targetBA);
        assertContent(cB.msgTargets, cBMsgTargets);
        
        // Bus

        HashSet<BusUser> targetCA = new HashSet<>();
        targetCA.add(cA);
        HashSet<BusUser> targetCB = new HashSet<>();
        targetCB.add(cB);

        HashMap<MessageInformation, HashSet<BusUser>> bATargets = new HashMap<>();
        bATargets.put(mA, targetCB);
        bATargets.put(mB, targetCA);
        assertContent(bA.getMsgTargets(), bATargets);


        //TODO Test message transmission?
    }

    private <T> void assertContent(HashMap<MessageInformation, List<T>> map, HashMap<MessageInformation, HashSet<T>> reference){
        Assert.assertEquals(reference.size(), map.size());
        for (HashMap.Entry<MessageInformation, List<T>> e : map.entrySet()){
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