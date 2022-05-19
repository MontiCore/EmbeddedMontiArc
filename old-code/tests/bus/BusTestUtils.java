/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */

package simulation.bus;

import de.rwth.montisim.commons.controller.commons.BusEntry;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.BeforeClass;
import simulation.EESimulator.Bridge;
import simulation.EESimulator.EEComponent;
import simulation.EESimulator.EESimulator;
import simulation.EESimulator.TestComponent;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static org.junit.Assert.assertTrue;


public class BusTestUtils {

    private static Map<Integer, BusEntry> busEntryByOrdinal = new HashMap<Integer, BusEntry>();

    public static void init() {
        if (busEntryByOrdinal.size() == 0) {
            for (BusEntry entry : BusEntry.values()) {
                busEntryByOrdinal.put(entry.ordinal(), entry);
            }
        }
    }

    public static List<Bus> createBusStructure(EESimulator eeSimulator) {
        List<EEComponent> comps1 = new ArrayList<EEComponent>();
        List<EEComponent> comps2 = new ArrayList<EEComponent>();
        List<EEComponent> comps3 = new ArrayList<EEComponent>();
        List<Bus> buses = new ArrayList<>();

        int i = 0;
        for (; i < 5; i++) {
            comps1.add(new TestComponent(eeSimulator));
        }
        for (; i < 8; i++) {
            comps2.add(new TestComponent(eeSimulator));
        }
        for (; i < 15; i++) {
            comps3.add(new TestComponent(eeSimulator));
        }

        Bus bus1 = new FlexRay(eeSimulator);
        for (EEComponent component : comps1) {
            bus1.registerComponent(component);
        }
        buses.add(bus1);

        Bus bus2 = new CAN(eeSimulator, CANOperationMode.MEDIUM_SPEED_CAN);// subComponents1
        for (EEComponent component : comps2) {
            bus2.registerComponent(component);
        }
        buses.add(bus2);

        Bus bus3 = new InstantBus(eeSimulator);// mainComponents
        for (EEComponent component : comps3) {
            bus3.registerComponent(component);
        }
        buses.add(bus3);

        //add new buses to structure as they are implemented

        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus2), Duration.ofMillis(1));

        new Bridge(new ImmutablePair<Bus, Bus>(bus2, bus3), Duration.ZERO);

        new Bridge(new ImmutablePair<Bus, Bus>(bus1, bus3), Duration.ofNanos(200));

        return buses;
    }

    public static BusMessageEvent createNregisterMessage(Bus bus, Object message, int senderPos, int messageLength,
                                                         int priority) {
        assertTrue(busEntryByOrdinal.size() > priority);

        List<EEComponent> connectedComponents = bus.getConnectedComponents();
        assertTrue(senderPos >= 0 && senderPos < connectedComponents.size());

        BusMessageEvent msg = new BusMessageEvent(message, messageLength, busEntryByOrdinal.get(priority), Instant.EPOCH,
                connectedComponents.get(senderPos).getId(), bus);
        bus.registerMessage(msg);
        return msg;
    }

    public static List<BusMessageEvent> createNregisterMessages(Bus bus) {
        List<BusMessageEvent> msgs = new ArrayList<BusMessageEvent>();

        List<EEComponent> connectedComponents = bus.getConnectedComponents();

        // make sure ordering is deterministic => no two messages with same priority
        List<Integer> priorities = new ArrayList<Integer>();
        for (int j = 0; j < busEntryByOrdinal.size(); j++) {
            priorities.add(j);
        }

        Random rand = new Random();
        for (int j = 0; j < busEntryByOrdinal.size(); j++) {
            int senderPos = rand.nextInt(connectedComponents.size());
            int messageLength = rand.nextInt(1500) + 1;
            int priority = priorities.remove(rand.nextInt(priorities.size()));
            msgs.add(createNregisterMessage(bus, j, senderPos, messageLength, priority));
        }
        return msgs;
    }
}
