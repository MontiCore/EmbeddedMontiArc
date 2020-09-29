/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.ArrayType;
import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.MessageTypeManager;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class SetupErrorsTest {

    @Test
    public void cyclicSetup() throws EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);

        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"));
        b2.attachTo(eesystem);
        Bus b3 = new ConstantBus(ConstantBusProperties.instantBus().setName("b3"));
        b3.attachTo(eesystem);
        Bridge r1 = new Bridge(BridgeProperties.instantBridge().setName("r1"));
        r1.attachTo(eesystem);
        Bridge r2 = new Bridge(BridgeProperties.instantBridge().setName("r2"));
        r2.attachTo(eesystem);
        Bridge r3 = new Bridge(BridgeProperties.instantBridge().setName("r3"));
        r3.attachTo(eesystem);
        r1.connectToBus(b1);
        r1.connectToBus(b2);
        r2.connectToBus(b2);
        r2.connectToBus(b3);
        r3.connectToBus(b3);
        r3.connectToBus(b1);
        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e) {
            foundError = e.errors.cyclicError.isPresent();
            // e.printStackTrace();
        }
        Assert.assertTrue("Expected an EECyclicSetupException to be thrown.", foundError);
    }

    // Two components give the same output
    @Test
    public void sameOutput() throws EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"));
        b2.attachTo(eesystem);
        Bridge r1 = new Bridge(BridgeProperties.instantBridge().setName("r1"));
        r1.attachTo(eesystem);
        r1.connectToBus(b1);
        r1.connectToBus(b2);
        TestEEComponent c1 = new TestEEComponent("c1");
        c1.attachTo(eesystem);
        TestEEComponent c2 = new TestEEComponent("c2");
        c2.attachTo(eesystem);
        TestEEComponent c3 = new TestEEComponent("c3");
        c3.attachTo(eesystem);
        c1.connectToBus(b1);
        c2.connectToBus(b2);
        c3.connectToBus(b2);
        c1.addOutput("m1", BasicType.DOUBLE);
        c2.addOutput("m1", BasicType.DOUBLE);
        c3.addInput("m1", BasicType.DOUBLE);
        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e) {
            foundError = e.errors.multipleInputsExceptions.size() == 1;
            // e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMultipleInputsException to be thrown.", foundError);
    }

    // A required input of a component is missing
    @Test
    public void missingInput() throws EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"));
        b2.attachTo(eesystem);
        TestEEComponent c1 = new TestEEComponent("c1"); c1.attachTo(eesystem);
        TestEEComponent c2 = new TestEEComponent("c2"); c2.attachTo(eesystem);
        TestEEComponent c3 = new TestEEComponent("c3"); c3.attachTo(eesystem);
        TestEEComponent c4 = new TestEEComponent("c4"); c4.attachTo(eesystem);
        c1.connectToBus(b1);
        c2.connectToBus(b2);
        c3.connectToBus(b2);
        c1.addOutput("m1", BasicType.DOUBLE);
        c2.addInput("m1", BasicType.DOUBLE);
        c3.addInput("m2", BasicType.DOUBLE);
        c4.addInput("m1", BasicType.DOUBLE);
        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e){
            foundError = e.errors.missingOutputExceptions.size() == 3;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMissingOutputException to be thrown.", foundError);
    }
    
    // Two components register the same message with different types (output-input, input-input)
    @Test
    public void messageType() throws EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        TestEEComponent c1 = new TestEEComponent("c1"); c1.attachTo(eesystem);
        TestEEComponent c2 = new TestEEComponent("c2"); c2.attachTo(eesystem);
        TestEEComponent c3 = new TestEEComponent("c3"); c3.attachTo(eesystem);
        c1.connectToBus(b1);
        c2.connectToBus(b1);
        c3.connectToBus(b1);
        int errorsFound = 0;
        try {
            c1.addOutput("m1", BasicType.DOUBLE);
            c2.addInput("m1", BasicType.INT);
        } catch (EEMessageTypeException e){
            ++errorsFound;
        }
        try {
            c3.addInput("m1", new ArrayType(BasicType.INT, 5));
        } catch (EEMessageTypeException e){
            ++errorsFound;
        }
        Assert.assertEquals("Expected 2 EEMessageTypeExceptions to be thrown.", 2, errorsFound);
    }

    // A component is connected to a bus using an invalid name or id
    @Test
    public void invalidConnect() throws EEMessageTypeException {
        MessageTypeManager mtManager = new MessageTypeManager();
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(), mtManager);
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"));
        b1.attachTo(eesystem);
        TestEEComponent c1 = new TestEEComponent("c1"); c1.attachTo(eesystem);
        TestEEComponent c2 = new TestEEComponent("c2"); c2.attachTo(eesystem);
        TestEEComponent c3 = new TestEEComponent("c3"); c3.attachTo(eesystem);
        c1.connectToBus("b2");
        c2.connectToBus(-1);
        c3.connectToBus(10);
        c2.connectToBus(1);
        c2.connectToBus("c1");
        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e){
            foundError = true;
            Assert.assertEquals("ComponentTypeException Count", 2, e.errors.componentTypeExceptions.size());
            Assert.assertEquals("InvalidIdException Count", 2, e.errors.invalidIdExceptions.size());
            Assert.assertEquals("MissingComponentException Count", 1, e.errors.missingComponentExceptions.size());
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected errors to be thrown.", foundError);
    }
}