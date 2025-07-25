/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package simulation.bus;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.time.Duration;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;
import java.util.UUID;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.BeforeClass;
import org.junit.Test;

import com.google.common.base.Function;
import com.google.common.base.Predicate;
import com.google.common.collect.Iterables;
import com.google.common.collect.Lists;

import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EEComponentType;
import simulation.EESimulator.EEDiscreteEvent;
import simulation.EESimulator.EEDiscreteEventTypeEnum;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.TestComponent;

public class BusTest {

    private static EESimulator EEsim = new EESimulator(Instant.EPOCH);

    private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

    @BeforeClass
    public static void oneTimeSetUp() {
        for (BusEntry entry : BusEntry.values()) {
            busEntryByOrdinal.put(entry.ordinal(), entry);
        }
    }

    @Test
    public void testSetKeepAlive() {
        EEsim = new EESimulator(Instant.EPOCH);
        Bus bus = createBusStructure();

        // no messages => keepAlive should not set a new keepAlive
        bus.setKeepAlive();
        assertTrue(EEsim.getEventList().isEmpty());

        createNregisterMessage(bus, "finished second", 0, 3000, 10);
        bus.setKeepAlive();
        assertTrue(EEsim.getEventList().size() == 1);
        assertEquals(EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT, EEsim.getEventList().peek().getEventType());
        assertEquals(bus, EEsim.getEventList().peek().getTarget());

        // new keepAlive not added since finished after old keepAlive
        UUID oldKeepAliveId = EEsim.getEventList().peek().getId();
        createNregisterMessage(bus, "finished third", 0, 1500, 0);
        bus.setKeepAlive();
        assertTrue(EEsim.getEventList().size() == 1);
        assertEquals(oldKeepAliveId, EEsim.getEventList().peek().getId());

        // new keepAlive added since finished before old keepAlive
        Instant oldKeepAliveTime = EEsim.getEventList().peek().getEventTime();
        createNregisterMessage(bus, "finished first", 0, 1500, 20);
        bus.setKeepAlive();
        assertTrue(EEsim.getEventList().size() == 2);
        EEDiscreteEvent newKeepAlive = EEsim.getEventList().poll();
        EEDiscreteEvent oldKeepAlive = EEsim.getEventList().poll();
        assertTrue(oldKeepAliveTime.isAfter(newKeepAlive.getEventTime()));
        assertEquals(oldKeepAliveId, oldKeepAlive.getId());
    }

    @Test
    public void testBusSetup() {
        List<Bus> buses = createCyclicBusStructure();
        for (Bus bus : buses) {
            //get local messages
            Iterable<EEComponent> testComponents = Iterables.filter(bus.getConnectedComponents(),
                    new Predicate<EEComponent>() {
                        public boolean apply(EEComponent comp) {
                            return comp.getComponentType() == EEComponentType.TEST_COMPONENT;
                        }
                    });
            Function<EEComponent, List<BusEntry>> func = new Function<EEComponent, List<BusEntry>>() {
                @Override
                public List<BusEntry> apply(EEComponent comp) {
                    return comp.getSubscribedMessages();
                }
            };
            List<BusEntry> localSubscribedMessages = Lists
                    .newArrayList(Iterables.concat(Iterables.transform(testComponents, func)));

            assertEquals(4, bus.getSubscribedMessages().size());
            HashMap<BusEntry, List<EEComponent>> targetsByMessageId = bus.getTargetsByMessageId();
            assertEquals(4, targetsByMessageId.size());
            for (Map.Entry<BusEntry, List<EEComponent>> entry : targetsByMessageId.entrySet()) {
                if (localSubscribedMessages.contains(entry.getKey())) {
                    assertEquals(3, entry.getValue().size());
                } else {
                    assertEquals(2, entry.getValue().size());
                }
            }
        }
    }

    @Test
    public void testProcessEvents() {
        EEsim = new EESimulator(Instant.EPOCH);
        List<Bus> buses = createCyclicBusStructure();
        Map<UUID, Map<BusEntry, UUID>> messageMapByBusId = new HashMap<UUID, Map<BusEntry, UUID>>();

        for (Bus bus : buses) {
            Map<BusEntry, UUID> messagesByMessageId = new HashMap<BusEntry, UUID>();
            for (BusEntry messageId : bus.getSubscribedMessages()) {
                UUID message = UUID.randomUUID();
                bus.processEvent(new BusMessageEvent(message, 1, messageId, Instant.EPOCH,
                        bus.getConnectedComponents().get(0).getId(), bus));
                messagesByMessageId.put(messageId, message);
            }
            messageMapByBusId.put(bus.getId(), messagesByMessageId);
        }

        PriorityQueue<EEDiscreteEvent> eventsRef = EEsim.getEventList();
        PriorityQueue<EEDiscreteEvent> events = new PriorityQueue<EEDiscreteEvent>(eventsRef.comparator());

        assertEquals(3, eventsRef.size());

        // process keepAlive events
        while (!eventsRef.isEmpty()) {
            EEDiscreteEvent event = eventsRef.poll();
            if (event.getEventType() == EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT) {
                assertEquals(EEComponentType.BUS, event.getTarget().getComponentType());
                event.getTarget().processEvent(event);
            } else {
                assertEquals(EEDiscreteEventTypeEnum.BUSMESSAGE, event.getEventType());
                events.add(event);
            }
        }

        assertEquals(30, events.size());

        // process events at bridges and receivers
        while (!events.isEmpty()) {
            EEDiscreteEvent event = events.poll();
            assertEquals(EEDiscreteEventTypeEnum.BUSMESSAGE, event.getEventType());
            assertTrue(event.getTarget().getComponentType().toString(), event.getTarget().getComponentType() == EEComponentType.BRIDGE
                    || event.getTarget().getComponentType() == EEComponentType.TEST_COMPONENT);
            event.getTarget().processEvent(event);
        }

        while (!eventsRef.isEmpty()) {
            events.add(eventsRef.poll());
        }
        assertEquals(24, events.size());

        // process events at bus
        while (!events.isEmpty()) {
            EEDiscreteEvent event = events.poll();
            assertTrue(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE);
            assertTrue(event.getTarget().getComponentType() == EEComponentType.BUS);
            event.getTarget().processEvent(event);
        }

        assertEquals(3, eventsRef.size());

        // process keepAlive events
        while (!eventsRef.isEmpty()) {
            EEDiscreteEvent event = eventsRef.poll();
            if (event.getEventType() == EEDiscreteEventTypeEnum.KEEP_ALIVE_EVENT) {
                assertEquals(EEComponentType.BUS, event.getTarget().getComponentType());
                event.getTarget().processEvent(event);
            } else {
                assertEquals(EEDiscreteEventTypeEnum.BUSMESSAGE, event.getEventType());
                events.add(event);
            }
        }

        assertEquals(60, events.size());

        // process events at bridges and receivers
        while (!events.isEmpty()) {
            EEDiscreteEvent event = events.poll();
            assertTrue(event.getEventType() == EEDiscreteEventTypeEnum.BUSMESSAGE);
            assertTrue(event.getTarget().getComponentType() == EEComponentType.BRIDGE
                    || event.getTarget().getComponentType() == EEComponentType.TEST_COMPONENT);
            event.getTarget().processEvent(event);
        }

        while (!eventsRef.isEmpty()) {
            events.add(eventsRef.poll());
        }
        // after two hops the messages should not be processed anymore
        assertTrue(EEsim.getEventList().isEmpty());

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

    private List<Bus> createCyclicBusStructure() {
        Bus bus1 = new FlexRay(EEsim);
        Bus bus2 = new FlexRay(EEsim);
        Bus bus3 = new FlexRay(EEsim);

        TestComponent listenerEngine = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_ENGINE));
        TestComponent listenerGear = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_GEAR));
        TestComponent listenerBrake = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_BRAKE));
        TestComponent listnerSteering = new TestComponent(EEsim, Collections.singletonList(BusEntry.ACTUATOR_STEERING));

        // all buses want engine messages
        bus1.registerComponent(listenerEngine);
        bus2.registerComponent(listenerEngine);
        bus3.registerComponent(listenerEngine);

        bus1.registerComponent(listenerGear);
        bus2.registerComponent(listenerBrake);
        bus3.registerComponent(listnerSteering);

        // create cycle
        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus2), Duration.ofSeconds(2));
        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus3), Duration.ofSeconds(2));
        new Bridge(new ImmutablePair<Bus, Bus>(bus2, bus3), Duration.ofSeconds(2));

        List<Bus> buses = new ArrayList<Bus>();
        buses.add(bus1);
        buses.add(bus2);
        buses.add(bus3);
        return buses;
    }

    private FlexRay createBusStructure() {
        List<EEComponent> mainComponents = new ArrayList<EEComponent>();
        List<EEComponent> subComponents1 = new ArrayList<EEComponent>();
        List<EEComponent> subComponents2 = new ArrayList<EEComponent>();
        int i = 0;
        for (; i < 5; i++) {
            mainComponents.add(new TestComponent(EEsim));
        }
        for (; i < 8; i++) {
            subComponents1.add(new TestComponent(EEsim));
        }
        for (; i < 15; i++) {
            subComponents2.add(new TestComponent(EEsim));
        }

        FlexRay sub2 = new FlexRay(EEsim);// subComponents2
        for (EEComponent component : subComponents2) {
            sub2.registerComponent(component);
        }

        FlexRay sub1 = new FlexRay(EEsim);// subComponents1
        for (EEComponent component : subComponents1) {
            sub1.registerComponent(component);
        }

        new Bridge(new ImmutablePair<Bus, Bus>(sub1, sub2), Duration.ZERO);
        FlexRay main = new FlexRay(EEsim);// mainComponents
        for (EEComponent component : mainComponents) {
            main.registerComponent(component);
        }
        new Bridge(new ImmutablePair<Bus, Bus>(main, sub1), Duration.ZERO);
        return main;
    }

    private BusMessageEvent createNregisterMessage(Bus bus, Object message, int senderPos, int messageLength, int priority) {
        assertTrue(busEntryByOrdinal.size() > priority);

        List<EEComponent> connectedComponents = bus.getConnectedComponents();
        assertTrue(senderPos >= 0 && senderPos < connectedComponents.size());

        BusMessageEvent msg = new BusMessageEvent(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
                connectedComponents.get(senderPos).getId(), bus);
        bus.registerMessage(msg);
        return msg;
    }
}
