/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.network;

import de.rwth.montisim.commons.simulation.SimulationLoopExecutable;
import de.rwth.montisim.commons.utils.Vec3;
import org.junit.Test;
import simulation.network.settings.SettingsSimple;
import simulation.network.tasks.TaskAppTrafficOptimization;
import de.rwth.montisim.simulation.util.Log;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static org.junit.Assert.assertTrue;
import static simulation.network.tasks.TaskAppTrafficOptimization.TRAFFIC_OPTIMIZATION_MIN_COUNT;
import static simulation.network.tasks.TaskAppTrafficOptimization.TRAFFIC_OPTIMIZATION_MIN_TIME;
import static simulation.network.tasks.TaskAppTrafficOptimization.TRAFFIC_OPTIMIZATION_NOT_MOVED_GROUP_RANGE;

/**
 * Test for basic message soft state cleanup functionality
 */
public class TaskAppTrafficOptimizationTest {

    @Test
    public void testNoTrafficJamTime() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        // Ensure that no traffic jam is detected because there is one vehicle that is not standing for a long time
        Map<String, Map.Entry<Long, Vec3>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<Vec3> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            Vec3 pos = new Vec3(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, Vec3> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);

            // Change one value to prevent detection
            if (i == 0) {
                Map.Entry<Long, Vec3> modEntry = new AbstractMap.SimpleEntry<>(1L, pos);
                infoMap.put("fe899fe796f9fa75" + i, modEntry);
            }
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, TRAFFIC_OPTIMIZATION_MIN_TIME);
        TaskAppTrafficOptimization.trafficJamDetection(infoMap, outputPositionSet);

        assertTrue(outputPositionSet.isEmpty());

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testNoTrafficJamDistance() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        // Ensure that no traffic jam is detected because there is one vehicle that is too far away from traffic jam group
        Map<String, Map.Entry<Long, Vec3>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<Vec3> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            Vec3 pos = new Vec3(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, Vec3> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);

            // Change one value to prevent detection
            if (i == 0) {
                Vec3 modPos = new Vec3(new double[]{1000.00 - TRAFFIC_OPTIMIZATION_NOT_MOVED_GROUP_RANGE, 1000.00 + 2 * i, 0.50});
                Map.Entry<Long, Vec3> modEntry = new AbstractMap.SimpleEntry<>(0L, modPos);
                infoMap.put("fe899fe796f9fa75" + i, modEntry);
            }
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, TRAFFIC_OPTIMIZATION_MIN_TIME);
        TaskAppTrafficOptimization.trafficJamDetection(infoMap, outputPositionSet);

        assertTrue(outputPositionSet.isEmpty());

        // Enable log
        Log.setLogEnabled(true);
    }

    @Test
    public void testTrafficJamDetected() {
        // Setup
        Log.setLogEnabled(false);
        NetworkSettings settings = new SettingsSimple();

        NetworkSimulator.resetInstance();
        NetworkSimulator networkSimulator = NetworkSimulator.getInstance();
        networkSimulator.setNetworkSettings(settings);

        // Ensure that traffic jam is detected when all parameters are met
        Map<String, Map.Entry<Long, Vec3>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<Vec3> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            Vec3 pos = new Vec3(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, Vec3> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), Instant.EPOCH, TRAFFIC_OPTIMIZATION_MIN_TIME);
        TaskAppTrafficOptimization.trafficJamDetection(infoMap, outputPositionSet);

        assertTrue(!outputPositionSet.isEmpty());

        // Enable log
        Log.setLogEnabled(true);
    }
}
