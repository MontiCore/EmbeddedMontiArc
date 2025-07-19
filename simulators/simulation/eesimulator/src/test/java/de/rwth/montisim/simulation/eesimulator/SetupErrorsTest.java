/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eesimulator;

import java.time.Instant;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.VectorType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.simulation.eesimulator.bridge.Bridge;
import de.rwth.montisim.simulation.eesimulator.bridge.BridgeProperties;
import de.rwth.montisim.simulation.eesimulator.bus.Bus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBus;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;

public class SetupErrorsTest {


    // Two components give the same output
    @Test
    public void sameOutput() throws EEMessageTypeException {
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"), eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"), eesystem);
        Bridge r1 = new Bridge(BridgeProperties.instantBridge().setName("r1"), eesystem);
        r1.connectToComponent(b1);
        r1.connectToComponent(b2);
        TestEEComponent c1 = new TestEEComponent("c1", eesystem);
        TestEEComponent c2 = new TestEEComponent("c2", eesystem);
        TestEEComponent c3 = new TestEEComponent("c3", eesystem);
        c1.connectToComponent(b1);
        c2.connectToComponent(b2);
        c3.connectToComponent(b2);
        c1.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.DOUBLE));
        c2.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.DOUBLE));
        c3.addPort(PortInformation.newOptionalInputDataPort("m1", BasicType.DOUBLE, false));
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
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"), eesystem);
        Bus b2 = new ConstantBus(ConstantBusProperties.instantBus().setName("b2"), eesystem);
        TestEEComponent c1 = new TestEEComponent("c1", eesystem);
        TestEEComponent c2 = new TestEEComponent("c2", eesystem);
        TestEEComponent c3 = new TestEEComponent("c3", eesystem);
        TestEEComponent c4 = new TestEEComponent("c4", eesystem);
        c1.connectToComponent(b1);
        c2.connectToComponent(b2);
        c3.connectToComponent(b2);
        c1.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.DOUBLE));
        c2.addPort(PortInformation.newRequiredInputDataPort("m1", BasicType.DOUBLE, false));
        c3.addPort(PortInformation.newRequiredInputDataPort("m2", BasicType.DOUBLE, false));
        c4.addPort(PortInformation.newRequiredInputDataPort("m1", BasicType.DOUBLE, false));
        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e) {
            foundError = e.errors.missingOutputExceptions.size() == 3;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMissingOutputException to be thrown.", foundError);
    }

    // Two components register the same message with different types (output-input, input-input)
    @Test
    public void messageType() throws EEMessageTypeException {
        EESystem eesystem = new EESystem(new DiscreteEventSimulator(Instant.EPOCH));
        Bus b1 = new ConstantBus(ConstantBusProperties.instantBus().setName("b1"), eesystem);
        TestEEComponent c1 = new TestEEComponent("c1", eesystem);
        TestEEComponent c2 = new TestEEComponent("c2", eesystem);
        TestEEComponent c3 = new TestEEComponent("c3", eesystem);
        c1.connectToComponent(b1);
        c2.connectToComponent(b1);
        c3.connectToComponent(b1);
        c1.addPort(PortInformation.newOptionalOutputDataPort("m1", BasicType.DOUBLE));
        c2.addPort(PortInformation.newRequiredInputDataPort("m1", BasicType.INT, false));
        c3.addPort(PortInformation.newRequiredInputDataPort("m1", new VectorType(BasicType.INT, 5), false));

        boolean foundError = false;
        try {
            eesystem.finalizeSetup();
        } catch (EESetupException e) {
            foundError = e.errors.msgTypeExceptions.size() == 1;
            //e.printStackTrace();
        }
        Assert.assertTrue("Expected an EEMessageTypeExceptions to be thrown.", foundError);
    }

}