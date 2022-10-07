/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.eecomponents.simple_network;

import java.time.Duration;
import java.time.Instant;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.junit.Assert;
import org.junit.Test;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.DataType;
import de.rwth.montisim.commons.dynamicinterface.PortInformation;
import de.rwth.montisim.commons.dynamicinterface.SimplePacketType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.simulation.TimeUpdate;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.eesimulator.bus.constant.ConstantBusProperties;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMissingComponentException;
import de.rwth.montisim.simulation.eesimulator.exceptions.EESetupException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestCompProperties;
import de.rwth.montisim.simulation.eesimulator.testcomponents.TestEEComponent;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

public class CommunicationTest {

    @Test
    public void commTest() throws EEMessageTypeException, EESetupException, EEMissingComponentException {
        Instant start_time = Instant.ofEpochSecond(10);
        DiscreteEventSimulator simulator = new DiscreteEventSimulator(start_time);

        SimpleNetworkProperties networkProperties = new SimpleNetworkProperties();
        networkProperties.car_transmission_range = 15;
        SimpleNetwork network = new SimpleNetwork(networkProperties, simulator);

        BuildContext context = new BuildContext();
        context.addObject(simulator);
        context.addObject(network);

        VehicleProperties prop1 = new VehicleProperties().setName("v1");
        VehicleProperties prop2 = new VehicleProperties().setName("v2");
        VehicleProperties prop3 = new VehicleProperties().setName("v3");
        VehicleProperties prop4 = new VehicleProperties().setName("v4");
        prop1.start_pos = Optional.of(new Vec2(1, 0));
        prop2.start_pos = Optional.of(new Vec2(5, 7));
        prop3.start_pos = Optional.of(new Vec2(7, 1));
        prop4.start_pos = Optional.of(new Vec2(20, 7));

        DataType intPacket = new SimplePacketType(BasicType.INT);

        prop1.addComponent(
                ConstantBusProperties.instantBus().setName("DefaultBus")
        );
        prop1.addComponent(
                new SCGProperties().connectTo("DefaultBus")
        );
        prop1.addComponent(
                new TestCompProperties("TestComp1")
                        .addPort(PortInformation.newSocketPort("m1", intPacket, true, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .addPort(PortInformation.newSocketPort("m2", intPacket, false, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .addPort(PortInformation.newSocketPort("m3", intPacket, false, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .connectTo("DefaultBus")
        );
        prop2.addComponent(
                ConstantBusProperties.instantBus().setName("DefaultBus")
        );
        prop2.addComponent(
                new SCGProperties().connectTo("DefaultBus")
        );
        prop2.addComponent(
                new TestCompProperties("TestComp2")
                        .addPort(PortInformation.newSocketPort("m1", intPacket, true, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .addPort(PortInformation.newSocketPort("m2", intPacket, true, false).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .connectTo("DefaultBus")
        );
        prop3.addComponent(
                ConstantBusProperties.instantBus().setName("DefaultBus")
        );
        prop3.addComponent(
                new SCGProperties().connectTo("DefaultBus")
        );
        prop3.addComponent(
                new TestCompProperties("TestComp3")
                        .addPort(PortInformation.newSocketPort("m1", intPacket, true, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .addPort(PortInformation.newSocketPort("m2", intPacket, false, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .connectTo("DefaultBus")
        );
        prop4.addComponent(
                ConstantBusProperties.instantBus().setName("DefaultBus")
        );
        prop4.addComponent(
                new SCGProperties().connectTo("DefaultBus")
        );
        prop4.addComponent(
                new TestCompProperties("TestComp4")
                        .addPort(PortInformation.newSocketPort("m1", intPacket, true, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .addPort(PortInformation.newSocketPort("m2", intPacket, false, true).addTag(SimpleCommunicationGateway.NETWORK_TAG))
                        .connectTo("DefaultBus")
        );

        Vehicle v1 = prop1.build(context);
        Vehicle v2 = prop2.build(context);
        Vehicle v3 = prop3.build(context);
        Vehicle v4 = prop4.build(context);

        TestEEComponent testComp1 = (TestEEComponent) v1.eesystem.getComponent("TestComp1").orElseThrow(() -> new NoSuchElementException());
        TestEEComponent testComp2 = (TestEEComponent) v2.eesystem.getComponent("TestComp2").orElseThrow(() -> new NoSuchElementException());
        TestEEComponent testComp3 = (TestEEComponent) v3.eesystem.getComponent("TestComp3").orElseThrow(() -> new NoSuchElementException());
        TestEEComponent testComp4 = (TestEEComponent) v4.eesystem.getComponent("TestComp4").orElseThrow(() -> new NoSuchElementException());

        IPV6Address v1addr = ((SimpleCommunicationGateway) v1.eesystem.getComponent(SCGProperties.COMPONENT_NAME).orElseThrow(() -> new NoSuchElementException())).address;


        // Send messages
        testComp1.sendMessage(
                start_time.plus(Duration.ofMillis(5)),
                new Message(testComp1.getMsgInfo("m1"), SimpleNetworkMessage.newMessage(IPV6Address.BROADCAST_ADDR, Integer.valueOf(1)))
        );

        testComp2.sendMessage(
                start_time.plus(Duration.ofMillis(25)),
                new Message(testComp2.getMsgInfo("m1"), SimpleNetworkMessage.newMessage(v1addr, Integer.valueOf(2)))
        );
        testComp3.sendMessage(
                start_time.plus(Duration.ofMillis(35)),
                new Message(testComp3.getMsgInfo("m1"), SimpleNetworkMessage.newMessage(v1addr, Integer.valueOf(3)))
        );
        testComp4.sendMessage(
                start_time.plus(Duration.ofMillis(45)),
                new Message(testComp4.getMsgInfo("m1"), SimpleNetworkMessage.newMessage(v1addr, Integer.valueOf(4)))
        );


        testComp1.sendMessage(
                start_time.plus(Duration.ofMillis(70)),
                new Message(testComp1.getMsgInfo("m2"), SimpleNetworkMessage.newMessage(IPV6Address.BROADCAST_ADDR, Integer.valueOf(5)))
        );

        testComp1.sendMessage(
                start_time.plus(Duration.ofMillis(100)),
                new Message(testComp1.getMsgInfo("m3"), SimpleNetworkMessage.newMessage(IPV6Address.BROADCAST_ADDR, Integer.valueOf(6)))
        );

        simulator.update(new TimeUpdate(start_time, Duration.ofSeconds(1)));

        Assert.assertEquals(2, testComp1.events.size());
        Assert.assertEquals(2, testComp2.events.size());
        Assert.assertEquals(1, testComp3.events.size());
        Assert.assertEquals(0, testComp4.events.size());

        Assert.assertEquals(2, (long) (Integer) ((Object[]) testComp1.events.get(0).getMessage().message)[1]);
        Assert.assertEquals(3, (long) (Integer) ((Object[]) testComp1.events.get(1).getMessage().message)[1]);

        Assert.assertEquals(1, (long) (Integer) ((Object[]) testComp2.events.get(0).getMessage().message)[1]);
        Assert.assertEquals(5, (long) (Integer) ((Object[]) testComp2.events.get(1).getMessage().message)[1]);

        Assert.assertEquals(1, (long) (Integer) ((Object[]) testComp3.events.get(0).getMessage().message)[1]);
    }
}
