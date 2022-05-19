/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import org.junit.Before;
import org.junit.Test;
import de.rwth.montisim.simulation.environment.visualisationadapter.implementation.TrafficSignalImpl;

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
