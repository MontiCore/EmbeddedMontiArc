/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.EESimulator;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.Test;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.bus.*;

public class EESimulatorTest {

    @Test
    public void testSimulateNextTick() {
        EESimulator simulator = new EESimulator(Instant.EPOCH);

        Instant simTime = Instant.EPOCH;

        TestComponent listenerEngine = new TestComponent(simulator,
                Collections.singletonList(BusEntry.ACTUATOR_ENGINE)); // gets ACTUATOR_ENGINE
        TestComponent listenerGear = new TestComponent(simulator, Collections.singletonList(BusEntry.ACTUATOR_GEAR)); // gets
        // ACTUATOR_GEAR
        TestComponent listenerBrake = new TestComponent(simulator, Collections.singletonList(BusEntry.ACTUATOR_BRAKE)); // gets
        // ACTUATOR_BRAKE
        BusEntry listenTo[] = {BusEntry.ACTUATOR_STEERING, BusEntry.SENSOR_COMPASS};
        TestComponent listenerSteeringNCompass = new TestComponent(simulator, Arrays.asList(listenTo)); // gets
        // ACTUATOR_STEERING
        // and
        // SENSOR_COMPASS

        TestComponent transmitter = new TestComponent(simulator);

        FlexRay testBus = new FlexRay(simulator);
        testBus.registerComponent(listenerEngine);
        testBus.registerComponent(listenerGear);
        testBus.registerComponent(listenerBrake);
        testBus.registerComponent(listenerSteeringNCompass);
        testBus.registerComponent(transmitter);

        List<BusMessageEvent> messageList = new LinkedList<>();
        BusMessageEvent engineMsg = new BusMessageEvent("messageOne", 100, BusEntry.ACTUATOR_ENGINE, Instant.EPOCH.plusSeconds(3),
                transmitter.getId(), testBus);
        messageList.add(engineMsg);
        BusMessageEvent gearMsg = new BusMessageEvent("messageTwo", 200, BusEntry.ACTUATOR_GEAR, Instant.EPOCH.plusSeconds(32),
                transmitter.getId(), testBus);
        messageList.add(gearMsg);
        BusMessageEvent brakeMsg = new BusMessageEvent("messageThree", 20, BusEntry.ACTUATOR_BRAKE, Instant.EPOCH.plusSeconds(38),
                transmitter.getId(), testBus);
        messageList.add(brakeMsg);
        BusMessageEvent steeringMsg = new BusMessageEvent("messageFour", 150, BusEntry.ACTUATOR_STEERING,
                Instant.EPOCH.plusSeconds(4), transmitter.getId(), testBus);
        messageList.add(steeringMsg);
        BusMessageEvent engineMsg2 = new BusMessageEvent("messageFive", 30, BusEntry.ACTUATOR_ENGINE,
                Instant.EPOCH.plusSeconds(15), transmitter.getId(), testBus);
        messageList.add(engineMsg2);
        BusMessageEvent compassMsg = new BusMessageEvent("messageSix", 120, BusEntry.SENSOR_COMPASS,
                Instant.EPOCH.plusSeconds(40), transmitter.getId(), testBus);
        messageList.add(compassMsg);

        simulator.addEvent(engineMsg);
        simulator.addEvent(gearMsg);
        simulator.addEvent(steeringMsg);

        simulator.simulateNextTick(simTime);

        simTime = simTime.plusSeconds(30);

        simulator.addEvent(brakeMsg);
        simulator.addEvent(engineMsg2);

        simulator.simulateNextTick(simTime);

        assertTrue(listenerEngine.processedMessage(engineMsg.getMessage()));
        assertTrue(listenerSteeringNCompass.processedMessage(steeringMsg.getMessage()));
        assertTrue(listenerEngine.processedMessage(engineMsg2.getMessage()));

        simTime = simTime.plusSeconds(30);

        simulator.addEvent(compassMsg);

        simulator.simulateNextTick(simTime);

        assertTrue(listenerGear.processedMessage(gearMsg.getMessage()));
        assertTrue(listenerBrake.processedMessage(brakeMsg.getMessage()));
        assertTrue(listenerSteeringNCompass.processedMessage(compassMsg.getMessage()));
    }

    @Test
    public void testComplexBusStructure() {
        EESimulator EEsim = new EESimulator(Instant.EPOCH);
        List<Bus> buses = createComplexBusStrucutre(EEsim);

        Map<UUID, Map<BusEntry, UUID>> messageMapByBusId = new HashMap<UUID, Map<BusEntry, UUID>>();
        for (Bus bus : buses) {
            Map<BusEntry, UUID> messagesByMessageId = new HashMap<BusEntry, UUID>();
            for (BusEntry messageId : bus.getSubscribedMessages()) {
                UUID message = UUID.randomUUID();
                EEsim.addEvent(new BusMessageEvent(message, 20, messageId, Instant.EPOCH,
                        bus.getConnectedComponents().get(0).getId(), bus));
                messagesByMessageId.put(messageId, message);
            }
            messageMapByBusId.put(bus.getId(), messagesByMessageId);
        }

        EEsim.simulateNextTick(Instant.EPOCH.plusSeconds(30));

        for (Bus receiverBus : buses) {
            for (Bus senderBus : buses) {
                for (EEComponent comp : receiverBus.getConnectedComponents()) {
                    if (comp.getComponentType() == EEComponentType.TEST_COMPONENT) {
                        for (BusEntry messageId : comp.getSubscribedMessages()) {
                            UUID message = messageMapByBusId.get(senderBus.getId()).get(messageId);
                            assertTrue(((TestComponent) comp).processedMessage(message));
                        }

                    }
                }
            }
        }
    }

    private List<Bus> createComplexBusStrucutre(EESimulator EEsim) {
        Bus bus1 = new FlexRay(EEsim);
        Bus bus2 = new FlexRay(EEsim);
        Bus bus3 = new FlexRay(EEsim);
        Bus bus4 = new FlexRay(EEsim);

        TestComponent listenerEngine = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_ENGINE));
        TestComponent listenerGear = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_GEAR));
        TestComponent listenerBrake = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_BRAKE));
        TestComponent listnerSteering = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_STEERING));
        TestComponent listnerCarDetection = new TestComponent(EEsim, Collections.singletonList(BusEntry.COMPUTERVISION_DETECTED_CARS));

        // all cyclic buses want engine messages
        bus1.registerComponent(listenerEngine);
        bus2.registerComponent(listenerEngine);
        bus3.registerComponent(listenerEngine);

        bus1.registerComponent(listenerGear);
        bus2.registerComponent(listenerBrake);
        bus3.registerComponent(listnerSteering);
        bus4.registerComponent(listnerCarDetection);

        // create cycle
        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus2), Duration.ofMillis(2));
        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus3), Duration.ofMillis(2));
        new Bridge(new ImmutablePair<Bus, Bus>(bus2, bus3), Duration.ofMillis(2));
        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus4), Duration.ofMillis(2));

        List<Bus> buses = new ArrayList<Bus>();
        buses.add(bus1);
        buses.add(bus2);
        buses.add(bus3);
        buses.add(bus4);
        return buses;
    }
}
