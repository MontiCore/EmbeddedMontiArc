/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.simulation.eesimulator;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.ArrayType;
import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class SetupErrorsTest {

    @Test
    public void cyclicSetup(){
        EESimulator simulator = new EESimulator();
        Bus b1 = ConstantBus.newInstantBus(simulator, "b1");
        Bus b2 = ConstantBus.newInstantBus(simulator, "b2");
        Bus b3 = ConstantBus.newInstantBus(simulator, "b3");
        Bridge r1 = Bridge.newInstantBridge(simulator, "r1");
        Bridge r2 = Bridge.newInstantBridge(simulator, "r2");
        Bridge r3 = Bridge.newInstantBridge(simulator, "r3");
        r1.connectToBus(b1);
        r1.connectToBus(b2);
        r2.connectToBus(b2);
        r2.connectToBus(b3);
        r3.connectToBus(b3);
        r3.connectToBus(b1);
        boolean foundError = false;
        try {
            simulator.finalizeSetup();
        } catch (EESetupException e){
            foundError = e.errors.cyclicError.isPresent();
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EECyclicSetupException to be thrown.", foundError);
    }

    // Two components give the same output
    @Test
    public void sameOutput(){
        EESimulator simulator = new EESimulator();
        Bus b1 = ConstantBus.newInstantBus(simulator, "b1");
        Bus b2 = ConstantBus.newInstantBus(simulator, "b2");
        Bridge r1 = Bridge.newInstantBridge(simulator, "r1");
        r1.connectToBus(b1);
        r1.connectToBus(b2);
        TestEEComponent c1 = new TestEEComponent(simulator, "c1");
        TestEEComponent c2 = new TestEEComponent(simulator, "c2");
        TestEEComponent c3 = new TestEEComponent(simulator, "c3");
        c1.connectToBus(b1);
        c2.connectToBus(b2);
        c3.connectToBus(b2);
        c1.addOutput("m1", DataType.DOUBLE);
        c2.addOutput("m1", DataType.DOUBLE);
        c3.addOutput("m1", DataType.DOUBLE);
        boolean foundError = false;
        try {
            simulator.finalizeSetup();
        } catch (EESetupException e){
            foundError = e.errors.multipleInputsExceptions.size() == 1;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEOutputOverlapException to be thrown.", foundError);
    }

    // A required input of a component is missing
    @Test
    public void missingInput(){
        EESimulator simulator = new EESimulator();
        Bus b1 = ConstantBus.newInstantBus(simulator, "b1");
        Bus b2 = ConstantBus.newInstantBus(simulator, "b2");
        TestEEComponent c1 = new TestEEComponent(simulator, "c1");
        TestEEComponent c2 = new TestEEComponent(simulator, "c2");
        TestEEComponent c3 = new TestEEComponent(simulator, "c3");
        TestEEComponent c4 = new TestEEComponent(simulator, "c4");
        c1.connectToBus(b1);
        c2.connectToBus(b2);
        c3.connectToBus(b2);
        c1.addOutput("m1", DataType.DOUBLE);
        c2.addInput("m1", DataType.DOUBLE);
        c3.addInput("m2", DataType.DOUBLE);
        c4.addInput("m1", DataType.DOUBLE);
        boolean foundError = false;
        try {
            simulator.finalizeSetup();
        } catch (EESetupException e){
            foundError = e.errors.missingOutputExceptions.size() == 3;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMissingOutputException to be thrown.", foundError);
    }
    
    // Two components register the same message with different types (output-input, input-input)
    @Test
    public void messageType(){
        EESimulator simulator = new EESimulator();
        Bus b1 = ConstantBus.newInstantBus(simulator, "b1");
        TestEEComponent c1 = new TestEEComponent(simulator, "c1");
        TestEEComponent c2 = new TestEEComponent(simulator, "c2");
        TestEEComponent c3 = new TestEEComponent(simulator, "c3");
        c1.connectToBus(b1);
        c2.connectToBus(b1);
        c3.connectToBus(b1);
        c1.addOutput("m1", DataType.DOUBLE);
        c2.addInput("m1", DataType.INT);
        c3.addInput("m1", new ArrayType(DataType.INT, ArrayType.Dimensionality.ARRAY, 5));
        boolean foundError = false;
        try {
            simulator.finalizeSetup();
        } catch (EESetupException e){
            foundError = e.errors.messageTypeErrors.size() == 1;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMessageTypeException to be thrown.", foundError);
    }

    // A component is connected to a bus using an invalid name or id
    @Test
    public void invalidConnect(){
        EESimulator simulator = new EESimulator();
        ConstantBus.newInstantBus(simulator, "b1");
        TestEEComponent c1 = new TestEEComponent(simulator, "c1");
        TestEEComponent c2 = new TestEEComponent(simulator, "c2");
        TestEEComponent c3 = new TestEEComponent(simulator, "c3");
        c1.connectToBus("b2");
        c2.connectToBus(-1);
        c3.connectToBus(10);
        c2.connectToBus(1);
        c2.connectToBus("c1");
        boolean foundError = false;
        try {
            simulator.finalizeSetup();
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