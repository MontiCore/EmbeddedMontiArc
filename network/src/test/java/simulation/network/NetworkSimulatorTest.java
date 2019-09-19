/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.DiscreteEvent;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.junit.Test;

import simulation.EESimulator.EESimulator;
import simulation.bus.InstantBus;
import simulation.network.settings.SettingsSimple;
import simulation.util.Log;
import simulation.vehicle.PhysicalVehicle;
import simulation.vehicle.PhysicalVehicleBuilder;
import simulation.vehicle.Vehicle;
import simulation.vehicle.EEVehicle;
import simulation.vehicle.EEVehicleBuilder;
import simulation.vehicle.MassPointPhysicalVehicleBuilder;

import java.time.Duration;
import java.time.Instant;
import java.util.LinkedList;
import java.util.List;
import static org.junit.Assert.*;
import static simulation.network.NetworkDiscreteEventId.NETWORK_EVENT_ID_RANDOM_START_INITIALIZE;

/**
 * Test for basic network simulator functionality
 */
public class NetworkSimulatorTest {

    @Test
    public void testTimeAdvance() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        NetworkStatistics.resetInstance();
        NetworkStatistics statistics = NetworkStatistics.getInstance();

        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
		PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
		physicalVehicle.setPosition(new ArrayRealVector(new double[]{1000.0, 1000.0, 0.50}));
        NetworkNode networkNode = new NetworkNode(physicalVehicle);

        List<NetworkNode> nodeList = new LinkedList<>();
        nodeList.add(networkNode);
        networkSimulator.setNetworkNodes(nodeList);

        // Ensure that time advancement works
        assertTrue(networkSimulator.getSimulationTime().equals(Instant.EPOCH));
        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, Duration.ZERO);
        assertTrue(networkSimulator.getSimulationTime().equals(Instant.EPOCH));
        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, Duration.ofMillis(5));
        assertTrue(networkSimulator.getSimulationTime().equals(Instant.EPOCH.plusMillis(5)));

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testRandomTaskStart() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        NetworkStatistics.resetInstance();
        NetworkStatistics statistics = NetworkStatistics.getInstance();

        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
		PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
		physicalVehicle.setPosition(new ArrayRealVector(new double[]{1000.0, 1000.0, 0.50}));
        NetworkNode networkNode = new NetworkNode(physicalVehicle);

        List<NetworkNode> nodeList = new LinkedList<>();
        nodeList.add(networkNode);
        networkSimulator.setNetworkNodes(nodeList);

        // In the beginning there must be at least one event with NETWORK_EVENT_ID_RANDOM_START_INITIALIZE
        // between min/max task start time as specified in network settings
        assertTrue(!networkSimulator.getEventList().isEmpty());
        DiscreteEvent firstEvent = networkSimulator.getEventList().get(0);
        assertEquals(NETWORK_EVENT_ID_RANDOM_START_INITIALIZE.ordinal(), firstEvent.getEventId());
        assertTrue(!firstEvent.getEventTime().isBefore(settings.getMinTaskStartTime()));
        assertTrue(!firstEvent.getEventTime().isAfter(settings.getMaxTaskStartTime()));

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testRandomTaskStartConsumed() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        NetworkStatistics.resetInstance();
        NetworkStatistics statistics = NetworkStatistics.getInstance();

        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
		PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();
		physicalVehicle.setPosition(new ArrayRealVector(new double[]{1000.0, 1000.0, 0.50}));
        NetworkNode networkNode = new NetworkNode(physicalVehicle);

        List<NetworkNode> nodeList = new LinkedList<>();
        nodeList.add(networkNode);
        networkSimulator.setNetworkNodes(nodeList);

        // Ensure that first random start task is consumed and new events are produced
        assertTrue(!networkSimulator.getEventList().isEmpty());
        DiscreteEvent firstEvent = NetworkSimulator.getInstance().getEventList().get(0);
        assertTrue(networkSimulator.getEventList().size() == 1);

        if (!firstEvent.getEventTime().equals(Instant.EPOCH)) {
            networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, Duration.ZERO);
            assertTrue(!networkSimulator.getEventList().isEmpty());
            firstEvent = NetworkSimulator.getInstance().getEventList().get(0);
            assertEquals(NETWORK_EVENT_ID_RANDOM_START_INITIALIZE.ordinal(), firstEvent.getEventId());
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, Duration.between(Instant.EPOCH, settings.getMaxTaskStartTime()));
        assertTrue(!networkSimulator.getEventList().isEmpty());
        firstEvent = NetworkSimulator.getInstance().getEventList().get(0);
        assertNotEquals(NETWORK_EVENT_ID_RANDOM_START_INITIALIZE.ordinal(),firstEvent.getEventId());

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testStatisticsFunctions() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        NetworkStatistics.resetInstance();
        NetworkStatistics statistics = NetworkStatistics.getInstance();

        PhysicalVehicleBuilder physicalVehicleBuilder = new MassPointPhysicalVehicleBuilder();
		EESimulator eeSimulator = new EESimulator(Instant.EPOCH);
		EEVehicleBuilder eeVehicleBuilder = new EEVehicleBuilder(eeSimulator);
		InstantBus bus = new InstantBus(eeSimulator);
		eeVehicleBuilder.createAllSensorsNActuators(bus);
		Vehicle vehicle = new Vehicle(physicalVehicleBuilder, eeVehicleBuilder);
		PhysicalVehicle physicalVehicle = vehicle.getPhysicalVehicle();        
		physicalVehicle.setPosition(new ArrayRealVector(new double[]{1000.0, 1000.0, 0.50}));
        NetworkNode networkNode = new NetworkNode(physicalVehicle);

        List<NetworkNode> nodeList = new LinkedList<>();
        nodeList.add(networkNode);
        networkSimulator.setNetworkNodes(nodeList);

        // Ensure correct initial values
        assertTrue(statistics.getSentMessagesAmountPhy() == 0);
        assertTrue(statistics.getReceivedMessagesAmountApp() == 0);
        assertTrue(statistics.getReceiveInterruptionsPhy() == 0);
        assertTrue(statistics.getReceivedMessagesAmountLink() == 0);
        assertTrue(statistics.getSentMessagesAmountApp() == 0);

        // Ensure increasing values
        statistics.processSendMessageStartPhy(new NetworkMessage());
        statistics.processReceiveInterruptionPhy();
        assertTrue(statistics.getSentMessagesAmountPhy() == 1);
        assertTrue(statistics.getReceiveInterruptionsPhy() == 1);
        statistics.processSendMessageStartPhy(new NetworkMessage());
        statistics.processSendMessageStartPhy(new NetworkMessage());
        assertTrue(statistics.getSentMessagesAmountPhy() == 3);
        statistics.processReceivedMessageLink(new NetworkMessage());
        statistics.processReceivedMessageLink(new NetworkMessage());
        assertTrue(statistics.getReceivedMessagesAmountLink() == 2);
        statistics.processSendMessageApp(new NetworkMessage());
        statistics.processSendMessageApp(new NetworkMessage());
        assertTrue(statistics.getSentMessagesAmountApp() == 2);

        // Enable log
        Log.setLogEnabled(true);
    }
}
