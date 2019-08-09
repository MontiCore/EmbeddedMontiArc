/**
 *
 * ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package simulation.network;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.junit.Test;
import simulation.network.settings.SettingsSimple;
import simulation.network.tasks.TaskAppTrafficOptimization;
import simulation.util.Log;
import java.util.*;
import static org.junit.Assert.assertTrue;
import static simulation.network.tasks.TaskAppTrafficOptimization.TRAFFIC_OPTIMIZATION_MIN_COUNT;
import static simulation.network.tasks.TaskAppTrafficOptimization.TRAFFIC_OPTIMIZATION_MIN_TIME_NS;
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
        Map<String, Map.Entry<Long, RealVector>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<RealVector> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            RealVector pos = new ArrayRealVector(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, RealVector> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);

            // Change one value to prevent detection
            if (i == 0) {
                Map.Entry<Long, RealVector> modEntry = new AbstractMap.SimpleEntry<>(1L, pos);
                infoMap.put("fe899fe796f9fa75" + i, modEntry);
            }
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), 0, TRAFFIC_OPTIMIZATION_MIN_TIME_NS / 1000000L);
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
        Map<String, Map.Entry<Long, RealVector>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<RealVector> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            RealVector pos = new ArrayRealVector(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, RealVector> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);

            // Change one value to prevent detection
            if (i == 0) {
                RealVector modPos = new ArrayRealVector(new double[]{1000.00 - TRAFFIC_OPTIMIZATION_NOT_MOVED_GROUP_RANGE, 1000.00 + 2 * i, 0.50});
                Map.Entry<Long, RealVector> modEntry = new AbstractMap.SimpleEntry<>(0L, modPos);
                infoMap.put("fe899fe796f9fa75" + i, modEntry);
            }
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), 0, TRAFFIC_OPTIMIZATION_MIN_TIME_NS / 1000000L);
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
        Map<String, Map.Entry<Long, RealVector>> infoMap = Collections.synchronizedMap(new HashMap<>());
        Set<RealVector> outputPositionSet = Collections.synchronizedSet(new HashSet<>());

        for (int i = 0; i < TRAFFIC_OPTIMIZATION_MIN_COUNT; ++i) {
            RealVector pos = new ArrayRealVector(new double[]{1000.00 + 2 * i, 1000.00 + 2 * i, 0.50});
            Map.Entry<Long, RealVector> entry = new AbstractMap.SimpleEntry<>(0L, pos);
            infoMap.put("fe899fe796f9fa75" + i, entry);
        }

        networkSimulator.didExecuteLoop(new LinkedList<SimulationLoopExecutable>(), 0, TRAFFIC_OPTIMIZATION_MIN_TIME_NS / 1000000L);
        TaskAppTrafficOptimization.trafficJamDetection(infoMap, outputPositionSet);

        assertTrue(!outputPositionSet.isEmpty());

        // Enable log
        Log.setLogEnabled(true);
    }
}