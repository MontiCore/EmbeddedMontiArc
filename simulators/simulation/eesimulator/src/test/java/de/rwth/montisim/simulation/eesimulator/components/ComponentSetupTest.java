/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator.components;

import java.time.Instant;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.simulation.eesimulator.EEComponent;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMissingComponentException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class ComponentSetupTest {

    @Test
    public void singleBus() throws EESetupException, EEMessageTypeException, EEMissingComponentException {
        // Create an EE Setup.
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));
        Bus bus = new ConstantBus(ConstantBusProperties.instantBus().setName("TestInstantBus"), eesystem);
        TestEEComponent c1, c2, c3;
        c1 = new TestEEComponent("TestComponent1", eesystem);
        c2 = new TestEEComponent("TestComponent2", eesystem);
        c3 = new TestEEComponent("TestComponent3", eesystem);

        c1.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c1.addPort(PortInformation.newOptionalInputDataPort("m2", BasicType.DOUBLE, true));

        c2.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c2.addPort(PortInformation.newOptionalInputDataPort("m3", BasicType.INT, true));
        c2.addPort(PortInformation.newOptionalOutputDataPort("m4", BasicType.DOUBLE));
        c2.addPort(PortInformation.newOptionalOutputDataPort("m5", BasicType.DOUBLE));

        c3.addPort(PortInformation.newOptionalInputDataPort("m4", BasicType.DOUBLE, true));
        c3.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.INT));
        c3.addPort(PortInformation.newOptionalOutputDataPort("m2", BasicType.DOUBLE));
        c3.addPort(PortInformation.newOptionalOutputDataPort("m3", BasicType.INT));

        // Test different connection methods
        c1.connectToComponent(bus);
        c2.connectToComponent("TestInstantBus");
        c3.connectToComponent(bus);

        eesystem.finalizeSetup();

        // Validate Routing
        HashMap<MessageInformation, HashSet<EEComponent>> c1MsgTargets = new HashMap<>();
        assertContent(c1.msgRoutingTable, c1MsgTargets);

        HashSet<EEComponent> busTarget = new HashSet<>();
        busTarget.add(bus);

        HashMap<MessageInformation, HashSet<EEComponent>> c2MsgTargets = new HashMap<>();
        c2MsgTargets.put(c2.getMsgInfo("m4"), busTarget);
        assertContent(c2.msgRoutingTable, c2MsgTargets);

        HashMap<MessageInformation, HashSet<EEComponent>> c3MsgTargets = new HashMap<>();
        c3MsgTargets.put(c3.getMsgInfo("m1"), busTarget);
        c3MsgTargets.put(c3.getMsgInfo("m2"), busTarget);
        c3MsgTargets.put(c3.getMsgInfo("m3"), busTarget);
        assertContent(c3.msgRoutingTable, c3MsgTargets);

        HashMap<MessageInformation, HashSet<EEComponent>> busTargets = new HashMap<>();

        HashSet<EEComponent> m1Targets = new HashSet<>();
        m1Targets.add(c1);
        m1Targets.add(c2);
        busTargets.put(c3.getMsgInfo("m1"), m1Targets);

        HashSet<EEComponent> m2Targets = new HashSet<>();
        m2Targets.add(c1);
        busTargets.put(c3.getMsgInfo("m2"), m2Targets);

        HashSet<EEComponent> m3Targets = new HashSet<>();
        m3Targets.add(c2);
        busTargets.put(c3.getMsgInfo("m3"), m3Targets);

        HashSet<EEComponent> m4Targets = new HashSet<>();
        m4Targets.add(c3);
        busTargets.put(c2.getMsgInfo("m4"), m4Targets);

        assertContent(bus.msgRoutingTable, busTargets);

        //TODO Test message transmission?
    }

    @Test
    public void multiBus() throws EESetupException, EEMessageTypeException {
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));

        // Create an EE Setup.
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"), eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"), eesystem);
        Bus b3 = new ConstantBus(ConstantBusProperties.instantBus().setName("b3"), eesystem);
        Bus b4 = new ConstantBus(ConstantBusProperties.instantBus().setName("b4"), eesystem);
        Bus b5 = new ConstantBus(ConstantBusProperties.instantBus().setName("b5"), eesystem);
        TestEEComponent c1, c2, c3, c4, c5, c6;
        c1 = new TestEEComponent("c1", eesystem);
        c2 = new TestEEComponent("c2", eesystem);
        c3 = new TestEEComponent("c3", eesystem);
        c4 = new TestEEComponent("c4", eesystem);
        c5 = new TestEEComponent("c5", eesystem);
        c6 = new TestEEComponent("c6", eesystem);
        Bridge r1 = new Bridge(BridgeProperties.instantBridge().setName("r1"), eesystem);
        Bridge r2 = new Bridge(BridgeProperties.instantBridge().setName("r2"), eesystem);

        // Set ports

        // c.addPort(PortInformation.newOptionalOutputDataPort());
        // c.addPort(PortInformation.newOptionalInputDataPort(, true));

        c1.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.INT));
        c2.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c3.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c4.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c5.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        c6.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.INT, true));
        MessageInformation m1 = c1.getMsgInfo("m1");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m2", BasicType.DOUBLE));
        c2.addPort(PortInformation.newOptionalInputDataPort("m2", BasicType.DOUBLE, true));
        MessageInformation m2 = c1.getMsgInfo("m2");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m3", BasicType.INT));
        c3.addPort(PortInformation.newOptionalInputDataPort("m3", BasicType.INT, true));
        MessageInformation m3 = c1.getMsgInfo("m3");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m4", BasicType.DOUBLE));
        c4.addPort(PortInformation.newOptionalInputDataPort("m4", BasicType.DOUBLE, true));
        MessageInformation m4 = c1.getMsgInfo("m4");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m5", BasicType.INT));
        c4.addPort(PortInformation.newOptionalInputDataPort("m5", BasicType.INT, true));
        c5.addPort(PortInformation.newOptionalInputDataPort("m5", BasicType.INT, true));
        MessageInformation m5 = c1.getMsgInfo("m5");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m6", BasicType.DOUBLE));
        MessageInformation m6 = c1.getMsgInfo("m6");

        c1.addPort(PortInformation.newOptionalOutputDataPort("m7", BasicType.INT));
        c3.addPort(PortInformation.newOptionalInputDataPort("m7", BasicType.INT, true));
        c6.addPort(PortInformation.newOptionalInputDataPort("m7", BasicType.INT, true));
        MessageInformation m7 = c1.getMsgInfo("m7");

        // Connect components and bridges
        c1.connectToComponent(b1);
        c1.connectToComponent(b5);
        c2.connectToComponent(b1);
        c3.connectToComponent(b2);
        c4.connectToComponent(b4);
        c5.connectToComponent(b3);
        c6.connectToComponent(b5);

        r1.connectToComponent(b1);
        r1.connectToComponent(b2);
        r2.connectToComponent(b2);
        r2.connectToComponent(b3);
        r2.connectToComponent(b4);

        // Create "Parallel" Setup

        Bus bA = new ConstantBus(ConstantBusProperties.instantBus().setName("bA"), eesystem);
        TestEEComponent cA, cB;
        cA = new TestEEComponent("cA", eesystem);
        cB = new TestEEComponent("cB", eesystem);

        // Set ports
        cA.addPort(PortInformation.newOptionalOutputDataPort("mA", BasicType.INT));
        cB.addPort(PortInformation.newOptionalInputDataPort("mA", BasicType.INT, true));
        MessageInformation mA = cA.getMsgInfo("mA");

        cB.addPort(PortInformation.newOptionalOutputDataPort("mB", BasicType.DOUBLE));
        cA.addPort(PortInformation.newOptionalInputDataPort("mB", BasicType.DOUBLE, true));
        MessageInformation mB = cB.getMsgInfo("mB");

        // Connect components and bridges
        cA.connectToComponent(bA);
        cB.connectToComponent(bA);

        eesystem.finalizeSetup();

        // Validate Routing of main setup
        // Components

        HashSet<EEComponent> targetB1 = new HashSet<>();
        targetB1.add(b1);
        HashSet<EEComponent> targetB2 = new HashSet<>();
        targetB2.add(b2);
        HashSet<EEComponent> targetB4 = new HashSet<>();
        targetB4.add(b4);
        HashSet<EEComponent> targetB1B5 = new HashSet<>();
        targetB1B5.add(b1);
        targetB1B5.add(b5);
        HashSet<EEComponent> targetB3B4 = new HashSet<>();
        targetB3B4.add(b3);
        targetB3B4.add(b4);

        // C1
        HashMap<MessageInformation, HashSet<EEComponent>> c1MsgTargets = new HashMap<>();
        c1MsgTargets.put(m1, targetB1B5);
        c1MsgTargets.put(m2, targetB1);
        c1MsgTargets.put(m3, targetB1);
        c1MsgTargets.put(m4, targetB1);
        c1MsgTargets.put(m5, targetB1);
        c1MsgTargets.put(m7, targetB1B5);
        assertContent(c1.msgRoutingTable, c1MsgTargets);

        // C2, C3, C4, C5, C6
        assertContent(c2.msgRoutingTable, new HashMap<>());
        assertContent(c3.msgRoutingTable, new HashMap<>());
        assertContent(c4.msgRoutingTable, new HashMap<>());
        assertContent(c5.msgRoutingTable, new HashMap<>());
        assertContent(c6.msgRoutingTable, new HashMap<>());

        // Bridges
        HashMap<MessageInformation, HashSet<EEComponent>> r1MsgTargets = new HashMap<>();
        r1MsgTargets.put(m1, targetB2);
        r1MsgTargets.put(m3, targetB2);
        r1MsgTargets.put(m4, targetB2);
        r1MsgTargets.put(m5, targetB2);
        r1MsgTargets.put(m7, targetB2);
        assertContent(r1.msgRoutingTable, r1MsgTargets);

        HashMap<MessageInformation, HashSet<EEComponent>> r2MsgTargets = new HashMap<>();
        r2MsgTargets.put(m1, targetB3B4);
        r2MsgTargets.put(m4, targetB4);
        r2MsgTargets.put(m5, targetB3B4);
        assertContent(r2.msgRoutingTable, r2MsgTargets);

        // Buses

        HashSet<EEComponent> targetC2 = new HashSet<>();
        targetC2.add(c2);
        HashSet<EEComponent> targetC3 = new HashSet<>();
        targetC3.add(c3);
        HashSet<EEComponent> targetC4 = new HashSet<>();
        targetC4.add(c4);
        HashSet<EEComponent> targetC5 = new HashSet<>();
        targetC5.add(c5);
        HashSet<EEComponent> targetC6 = new HashSet<>();
        targetC6.add(c6);
        HashSet<EEComponent> targetR1 = new HashSet<>();
        targetR1.add(r1);
        HashSet<EEComponent> targetR2 = new HashSet<>();
        targetR2.add(r2);
        HashSet<EEComponent> targetR1C2 = new HashSet<>();
        targetR1C2.add(r1);
        targetR1C2.add(c2);
        HashSet<EEComponent> targetR2C3 = new HashSet<>();
        targetR2C3.add(r2);
        targetR2C3.add(c3);

        HashMap<MessageInformation, HashSet<EEComponent>> b1Targets = new HashMap<>();
        b1Targets.put(m1, targetR1C2);
        b1Targets.put(m2, targetC2);
        b1Targets.put(m3, targetR1);
        b1Targets.put(m4, targetR1);
        b1Targets.put(m5, targetR1);
        b1Targets.put(m7, targetR1);
        assertContent(b1.msgRoutingTable, b1Targets);

        HashMap<MessageInformation, HashSet<EEComponent>> b2Targets = new HashMap<>();
        b2Targets.put(m1, targetR2C3);
        b2Targets.put(m3, targetC3);
        b2Targets.put(m4, targetR2);
        b2Targets.put(m5, targetR2);
        b2Targets.put(m7, targetC3);
        assertContent(b2.msgRoutingTable, b2Targets);

        HashMap<MessageInformation, HashSet<EEComponent>> b3Targets = new HashMap<>();
        b3Targets.put(m1, targetC5);
        b3Targets.put(m5, targetC5);
        assertContent(b3.msgRoutingTable, b3Targets);

        HashMap<MessageInformation, HashSet<EEComponent>> b4Targets = new HashMap<>();
        b4Targets.put(m1, targetC4);
        b4Targets.put(m4, targetC4);
        b4Targets.put(m5, targetC4);
        assertContent(b4.msgRoutingTable, b4Targets);

        HashMap<MessageInformation, HashSet<EEComponent>> b5Targets = new HashMap<>();
        b5Targets.put(m1, targetC6);
        b5Targets.put(m7, targetC6);
        assertContent(b5.msgRoutingTable, b5Targets);

        // Validate Routing of Parallel Setup

        // Components
        HashSet<EEComponent> targetBA = new HashSet<>();
        targetBA.add(bA);

        HashMap<MessageInformation, HashSet<EEComponent>> cAMsgTargets = new HashMap<>();
        cAMsgTargets.put(mA, targetBA);
        assertContent(cA.msgRoutingTable, cAMsgTargets);

        HashMap<MessageInformation, HashSet<EEComponent>> cBMsgTargets = new HashMap<>();
        cBMsgTargets.put(mB, targetBA);
        assertContent(cB.msgRoutingTable, cBMsgTargets);

        // Bus

        HashSet<EEComponent> targetCA = new HashSet<>();
        targetCA.add(cA);
        HashSet<EEComponent> targetCB = new HashSet<>();
        targetCB.add(cB);

        HashMap<MessageInformation, HashSet<EEComponent>> bATargets = new HashMap<>();
        bATargets.put(mA, targetCB);
        bATargets.put(mB, targetCA);
        assertContent(bA.msgRoutingTable, bATargets);


        //TODO Test message transmission?
    }

    private void assertContent(HashMap<MessageInformation, List<EEComponent>> map, HashMap<MessageInformation, HashSet<EEComponent>> reference) {
        Assert.assertEquals(reference.size(), map.size());
        for (HashMap.Entry<MessageInformation, List<EEComponent>> e : map.entrySet()) {
            HashSet<EEComponent> s = reference.get(e.getKey());
            List<EEComponent> l = e.getValue();
            if (s == null && l == null) continue;
            Assert.assertNotNull(s);
            Assert.assertNotNull(l);
            Assert.assertEquals(s.size(), l.size());
            for (EEComponent t : l) {
                Assert.assertTrue("Object of list not in reference.", s.contains(t));
            }
        }
    }
}