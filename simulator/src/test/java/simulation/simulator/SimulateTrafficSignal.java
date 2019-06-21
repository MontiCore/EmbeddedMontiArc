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
package simulation.simulator;

import org.junit.Before;
import org.junit.Test;
import simulation.environment.visualisationadapter.implementation.TrafficSignalImpl;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static simulation.environment.visualisationadapter.interfaces.TrafficSignalStatus.*;

import java.time.Duration;

/**
 * Class that tests the TrafficSignalImpl class
 */
public class SimulateTrafficSignal {

    @Before
    public void setUp() {
        Simulator.resetSimulator();

        //Set update frequency to 30 loop iterations per second
        Simulator sim = Simulator.getSharedInstance();
    }

    @Test
    public void testLight() {
        Simulator simulator = Simulator.getSharedInstance();
        TrafficSignalImpl trafficSignal = new TrafficSignalImpl();
        simulator.registerSimulationObject(trafficSignal);

        // initially the signalA will be Green and signalB will be Red.
        assertTrue(trafficSignal.getSignalA() == GREEN);
        assertTrue(trafficSignal.getSignalB() == RED);

        // After 20 sec both signal will be yellow for 10 ms
        simulator.setSimulationDuration(Duration.ofMillis(22000));
        simulator.startSimulation();
        assertEquals(YELLOW.toString(), trafficSignal.getSignalA().toString());
        assertEquals(YELLOW.toString(), trafficSignal.getSignalB().toString());

        // After 30 sec signalA will be Red and the signalB will be Green.
        simulator.extendSimulationTime(Duration.ofMillis(10000));
        simulator.startSimulation();
        assertTrue(trafficSignal.getSignalA() == RED);
        assertTrue(trafficSignal.getSignalB() == GREEN);

        // After 50 sec both signal will be yellow for 10 ms
        simulator.extendSimulationTime(Duration.ofMillis(20000));
        simulator.startSimulation();
        assertTrue(trafficSignal.getSignalA() == YELLOW);
        assertTrue(trafficSignal.getSignalB() == YELLOW);


        // After 70s signalA will be Green and signalB will be Red again.
        simulator.extendSimulationTime(Duration.ofMillis(20000));
        simulator.startSimulation();
        assertTrue(trafficSignal.getSignalA() == GREEN);
        assertTrue(trafficSignal.getSignalB() == RED);
    }
}