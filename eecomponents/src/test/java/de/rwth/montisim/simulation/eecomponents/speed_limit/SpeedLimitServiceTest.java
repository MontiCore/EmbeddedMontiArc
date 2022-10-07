package de.rwth.montisim.simulation.eecomponents.speed_limit;

import de.rwth.montisim.commons.dynamicinterface.BasicType;
import de.rwth.montisim.commons.dynamicinterface.VectorType;
import de.rwth.montisim.commons.eventsimulation.DiscreteEventSimulator;
import de.rwth.montisim.commons.utils.BuildContext;
import de.rwth.montisim.commons.utils.Vec3;
import de.rwth.montisim.simulation.eecomponents.lidar.Lidar;
import de.rwth.montisim.simulation.eesimulator.EESystem;
import de.rwth.montisim.simulation.eesimulator.events.MessageReceiveEvent;
import de.rwth.montisim.simulation.eesimulator.exceptions.EEMessageTypeException;
import de.rwth.montisim.simulation.eesimulator.message.Message;
import de.rwth.montisim.simulation.eesimulator.message.MessageInformation;
import de.rwth.montisim.simulation.environment.world.World;
import de.rwth.montisim.simulation.environment.world.elements.Node;
import de.rwth.montisim.simulation.environment.world.elements.Way;

import org.junit.Assert;
import org.junit.Test;

import java.time.Instant;

import static org.mockito.Mockito.*;

public class SpeedLimitServiceTest {

    @Test
    public void test_fetchUpperSpeedLimits() throws EEMessageTypeException {

        World world = new World("Stub");

        Way way1 = new Way("way_1", true, 1, false, 30);
        Way way2 = new Way("way_2", true, 1, false, 50);
        Way way3 = new Way("way_3", true, 1, false, 30);

        world.addWay(way1);
        world.addWay(way2);
        world.addWay(way3);

        //first intersection between way1 and way2
        Node node1 = new Node(new Vec3(20, 0, 0));
        node1.ways.add(way1);
        node1.ways.add(way2);
        int node1Id = world.addNode(node1);

        //second intersection between way2 and way3
        Node node2 = new Node(new Vec3(50, 0, 0));
        node2.ways.add(way2);
        node2.ways.add(way3);
        int node2Id = world.addNode(node2);

        //waypoints of way1
        way1.addPoint(new Vec3(0, 0, 0), -1);
        way1.addPoint(new Vec3(10, 0, 0), -1);
        way1.addPoint(node1.point, node1Id);

        //waypoints of way2
        way2.addPoint(node1.point, node1Id);
        way2.addPoint(new Vec3(30, 0, 0), -1);
        way2.addPoint(new Vec3(40, 0, 0), -1);
        way2.addPoint(node2.point, node2Id);

        //waypoints of way3
        way3.addPoint(node2.point, node2Id);
        way3.addPoint(new Vec3(60, 0, 0), -1);
        way3.addPoint(new Vec3(70, 0, 0), -1);
        way3.addPoint(new Vec3(80, 0, 0), -1);
        way3.addPoint(new Vec3(90, 0, 0), -1);

        //construct SpeedlimitService with the world
        BuildContext buildContext = new BuildContext();
        buildContext.addObject(world, World.CONTEXT_KEY);
        DiscreteEventSimulator simulator = mock(DiscreteEventSimulator.class);
        EESystem eeSystem = new EESystem(simulator);
        SpeedLimitService speedLimitService = (SpeedLimitService) new SpeedLimitServiceProperties().build(eeSystem, buildContext);


        //the test trajectory with same points as on the individual ways
        double[] trajectoryX = new double[]{0, 10, 20, 30, 40, 50, 60, 70, 80, 90};
        double[] trajectoryY = new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        int trajectoryLength = 10;
        VectorType trajectoryDatatype = new VectorType(BasicType.DOUBLE, trajectoryLength);


        //send the test trajectory to the SpeedLimitService
        MessageInformation trajectoryLengthMessageInformation =
                new MessageInformation(speedLimitService.trajectoryLengthMsg, null, BasicType.N, null);
        MessageInformation trajectoryXMessageInformation =
                new MessageInformation(
                        speedLimitService.trajectoryXMsg,
                        null,
                        trajectoryDatatype,
                        null
                );
        MessageInformation trajectoryYMessageInformation =
                new MessageInformation(
                        speedLimitService.trajectoryYMsg,
                        null,
                        trajectoryDatatype,
                        null
                );
        Message trajectoryLengthMessage = new Message(trajectoryLengthMessageInformation, trajectoryLength);
        Message trajectoryXMessage = new Message(trajectoryXMessageInformation, trajectoryX, trajectoryLength);
        Message trajectoryYMessage = new Message(trajectoryYMessageInformation, trajectoryY, trajectoryLength);
        speedLimitService.receive(new MessageReceiveEvent(speedLimitService, Instant.now(), trajectoryLengthMessage));
        speedLimitService.receive(new MessageReceiveEvent(speedLimitService, Instant.now(), trajectoryXMessage));
        speedLimitService.receive(new MessageReceiveEvent(speedLimitService, Instant.now(), trajectoryYMessage));

        //assertion
        double[] expectedSpeedLimits = new double[]{30, 30, 50, 50, 50, 30, 30, 30, 30};
        double[] actualSpeedLimits = speedLimitService.fetchUpperSpeedLimits();
        Assert.assertArrayEquals(expectedSpeedLimits, actualSpeedLimits, Lidar.DOUBLE_TOLERANCE);
    }
}
