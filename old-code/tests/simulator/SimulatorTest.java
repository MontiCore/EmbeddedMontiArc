/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import de.rwth.montisim.commons.simulation.*;
import org.apache.commons.math3.linear.*;
import org.junit.*;
import de.rwth.montisim.simulation.util.*;

import java.time.Duration;
import java.time.Instant;
import java.util.*;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Class that tests the Simulator class
 */
public class SimulatorTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(true);
    }

    @After
    public void after() {
        Simulator sim = Simulator.getSharedInstance();
        try {
            sim.continueSimulation();
        } catch (IllegalStateException e) {
            // Simulation was apperently not paused
        }
        try {
            sim.waitUntilSimulationStopped();
        } catch (UnsupportedOperationException e) {
            // Simulation was apperently blocking
        }
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    //Test that do not need to start a simulation

    @Test(expected = IllegalStateException.class)
    public void startSimulationStartTwice() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(Duration.ofMillis(5000));
        sim.startSimulation();
        sim.startSimulation();
        System.out.println("startSimulationStartTwice succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void startSimulationSynchronousRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSynchronousSimulation(true);
        sim.startSimulation();
        System.out.println("startSimulationSynchronousRealTime succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSlowDownFactor(2);
        sim.startSimulation();
        System.out.println("startSimulationSlowedDownRealTime succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownMaxFPS() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_MAX_FPS);
        sim.setSlowDownFactor(2);
        sim.startSimulation();
        System.out.println("startSimulationSlowedDownMaxFPS succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownWallClockRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSlowDownWallClockFactor(2);
        sim.startSimulation();
        System.out.println("startSimulationSlowedDownWallClockRealTime succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownWallClockMaxFPS() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_MAX_FPS);
        sim.setSlowDownWallClockFactor(2);
        sim.startSimulation();
        System.out.println("startSimulationSlowedDownWallClockMaxFPS succesful");
    }

    @Test(expected = IllegalArgumentException.class)
    public void continueSimulationUntilNegative() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.setSimulationPauseTime(500);
        sim.startSimulation();
        sim.continueSimulation(Duration.ofMillis(-1));
        System.out.println("continueSimulationUntilNegative succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void continueSimulationUntilNotPaused() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();
        sim.continueSimulation(Duration.ofMillis(500));
        System.out.println("continueSimulationUntilNotPaused succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void continueSimulationUntilNotRunning() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.continueSimulation(Duration.ofMillis(500));
        System.out.println("continueSimulationUntilNotRunning succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void continueSimulationNotPaused() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();
        sim.continueSimulation();
        System.out.println("continueSimulationNotPaused succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void continueSimulationNotRunning() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.continueSimulation();
        System.out.println("continueSimulationNotRunning succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void stopSimulationNotRunning() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.stopSimulation();
        System.out.println("stopSimulationNotRunning succesful");
    }

    @Test(expected = IllegalStateException.class)
    public void setSimulationTypeRunning() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.setSimulationPauseTime(500);
        sim.startSimulation();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_FIXED_TIME);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setSimulationDurationInPast() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.setSimulationPauseTime(500);
        sim.startSimulation();
        sim.setSimulationDuration(Duration.ofMillis(250));
    }

    @Test(expected = IllegalArgumentException.class)
    public void extendSimulationTimeNegative() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.extendSimulationTime(Duration.ofMillis(-5));
    }

    @Test(expected = IllegalArgumentException.class)
    public void setSimulationLoopFrequencyNonPositive() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationLoopFrequency(0);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setSlowDownFactorLessThenOne() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSlowDownFactor(0);
    }

    @Test(expected = IllegalStateException.class)
    public void setSlowDownFactorAlreadyWallClock() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSlowDownWallClockFactor(2);
        sim.setSlowDownFactor(2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setSlowDownWallClockFactorLessThenOne() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSlowDownWallClockFactor(0);
    }

    @Test(expected = IllegalStateException.class)
    public void setSlowWallClockDownFactorAlreadySlowDown() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSlowDownFactor(2);
        sim.setSlowDownWallClockFactor(2);
    }

    @Test(expected = IllegalStateException.class)
    public void setStartDaytimeNull() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.setSimulationPauseTime(500);
        sim.startSimulation();
        sim.setStartDaytime(Calendar.getInstance().getTime());
    }

    @Test(expected = IllegalArgumentException.class)
    public void setDaytimeSpeedUpLessThanOne() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setDaytimeSpeedUp(0);
    }

    @Test(expected = IllegalStateException.class)
    public void setDaytimeSpeedUpNotPaused() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();
        sim.setDaytimeSpeedUp(2);
    }

    @Test(expected = IllegalArgumentException.class)
    public void setSimulationPauseTimeNegative() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationPauseTime(-1);
    }

    @Test(expected = IllegalStateException.class)
    public void resetErrorOccurredNotPaused() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();
        sim.resetErrorOccurred();
    }

    @Test(expected = IllegalArgumentException.class)
    public void registerSimulationObjectPhysicalObject() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.registerSimulationObject(new TestPhysicalObject());
    }

    @Test(expected = IllegalArgumentException.class)
    public void unregisterSimulationObject() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.unregisterSimulationObject(new TestPhysicalObject());
    }

    @Test(expected = UnsupportedOperationException.class)
    public void waitForSync() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.waitFor(Duration.ofMillis(500));
    }

    @Test(expected = UnsupportedOperationException.class)
    public void waitUntilSimulationStoppedSync() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.waitUntilSimulationStopped();
    }

    //Tests that need to start a simulation to test

    /**
     * Running the same fixed time simulation twice should lead to the same amount of simulated time and number loop iterations
     */
    @Test
    public void fixedTimeSimulationIsDeterministic() {
        // Start first run
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();

        // Get simulated time and loop count
        Instant simulationTime1 = sim.getSimulationTime();
        long loopCount1 = sim.getLoopCount();

        // Start second run
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(1000));
        sim.startSimulation();

        // Get simulated time and loop count
        Instant simulationTime2 = sim.getSimulationTime();
        long loopCount2 = sim.getLoopCount();

        // Check if simulation time and loop count are equal
        assertEquals(simulationTime1, simulationTime2);
        assertEquals(loopCount1, loopCount2);
    }

    /**
     * Fixed time simulations should simulate for the specified duration
     * with the expected number of loop iterations and step size
     */
    @Test
    public void fixedTimeIntervals() {
        // Calculate expected values
        int simulationLoopFrequency = 30;
        Duration simulationDuration = Duration.ofMillis(5000);
        Duration expectedIterationTime = Duration.ofMillis((long) ((1.0 / simulationLoopFrequency) * 1000));
        long expectedLoopCount = (simulationDuration.toMillis() / expectedIterationTime.toMillis()) + 1;

        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationLoopFrequency(simulationLoopFrequency);

        // Create and register observer that checks the step sizes
        StepSizeChecker checker = new StepSizeChecker(expectedIterationTime);
        sim.registerLoopObserver(checker);

        // Run simulation
        sim.startSimulation();

        Instant expectedTime = Instant.ofEpochMilli(simulationDuration.toMillis());
        // Check if expected simulation duration was met
        checkSimTime(sim, expectedTime);
        // Check if expected number of iterations were performed
        assertEquals(expectedLoopCount, sim.getLoopCount());
    }

    /**
     * Real time simulations should actually takes real time
     */
    @Test
    public void realTimeSimulationTakesRealTime() {
        // Calculate expected values
        long simulationDuration = 2000;

        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSimulationDuration(Duration.ofMillis(simulationDuration));

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start simulator
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if runtime is within 10% deviance of the simulationDuration
        assertTrue(simulationDuration * 0.9 <= runtime);
        assertTrue(runtime <= simulationDuration * 1.1);
    }

    /**
     * Max FPS simulations should actually takes real time
     */
    @Test
    public void maxFPSSimulationTakesRealTime() {
        // Calculate expected values
        long simulationDuration = 2000;

        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_MAX_FPS);
        sim.setSimulationDuration(Duration.ofMillis(simulationDuration));

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start simulator
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if runtime is within 10% deviance of the simulationDuration
        assertTrue(simulationDuration * 0.9 <= runtime);
        assertTrue(runtime <= simulationDuration * 1.1);
    }

    /**
     * Continuing a paused synchronous simulation for a specified time should pause the simulation after the specified tine
     */
    @Test
    public void continueSimulationUntilSync() {
        // Calculate expected values
        long continueSimulationTime = 500;
        long simulationPauseTime = 500;

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(Duration.ofMillis(5000));
        sim.setSimulationPauseTime(simulationPauseTime);
        sim.startSimulation();

        // Continue simulation for the specified time
        sim.continueSimulation(Duration.ofMillis(continueSimulationTime));

        Instant minimumSimulationTime = Instant.ofEpochMilli(continueSimulationTime + simulationPauseTime);
        // Check if simulation was continued for the specified time
        assertTrue(!minimumSimulationTime.isAfter(sim.getSimulationTime()));
        assertTrue(sim.getSimulationTime().isBefore(minimumSimulationTime.plusMillis(66)));
    }

    //TODO: Continuing a paused asynchronous simulation for a specified time should pause the simulation after the specified tine

    /**
     * Continuing a paused synchronous simulation should continue the simulation until the end
     */
    @Test
    public void continueSimulationSync() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Continue simulation for the specified time
        sim.continueSimulation();

        Instant expected = Instant.ofEpochMilli(simulationDuration.toMillis());
        // Check if simulation was continued for the specified time
        assertTrue(!expected.isAfter(sim.getSimulationTime()));
        assertTrue(sim.getSimulationTime().isBefore(expected.plusMillis(66)));
    }

    //TODO: Continuing a paused synchronous simulation should continue the simulation until the end

    /**
     * The set simulation duration should be met
     */
    @Test
    public void setSimulationDuration() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.startSimulation();

        Instant expectedTime = Instant.ofEpochMilli(simulationDuration.toMillis());
        // Check if simulation duration was met
        checkSimTime(sim, expectedTime);
    }

    /**
     * A simulation with 0 ms simulation duration should stop immediately
     */
    @Test
    public void setSimulationDurationZero() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(0);

        // Set up simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);

        // Create and register observer
        TestObserver checker = new TestObserver();
        sim.registerLoopObserver(checker);

        // Run simulation
        sim.startSimulation();

        // Check if simulation stopped immediately
        assertEquals(Instant.EPOCH, sim.getSimulationTime());
        // Check if observer was not notified
        assertEquals(0, checker.startCounter);
        assertEquals(0, checker.willExecCounter);
        assertEquals(0, checker.didExecCounter);
        assertEquals(0, checker.stopCounter);
    }

    /**
     * Extending the simulation time should extend the simulation duration
     */
    @Test
    public void extendSimulationTime() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        Duration simulationExtensionTime = Duration.ofMillis(500);

        // Setup and run simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Extend simulation time and continue
        sim.extendSimulationTime(simulationExtensionTime);
        sim.continueSimulation();

        Instant expectedTime = Instant.ofEpochMilli(simulationDuration.toMillis() + simulationExtensionTime.toMillis());
        // Check if simulation was extended successfully
        checkSimTime(sim, expectedTime);
    }

    /**
     * The slow down factor should have an effect on the runtime of a synchronous simulation
     */
    @Test
    public void setSlowDownFactorSync() {
        // Set up reference run
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        Duration simulationDuration = Duration.ofMillis(5000);
        sim.setSimulationDuration(simulationDuration);
        sim.registerSimulationObject(new SlowExecutable());

        // Remember reference start time
        long referenceStartTime = System.currentTimeMillis();

        // Start reference rum
        sim.startSimulation();

        // Calculate reference runtime
        long referenceRuntime = System.currentTimeMillis() - referenceStartTime;

        // Set up slowed down run
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSlowDownFactor(3);
        sim.registerSimulationObject(new SlowExecutable());

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start slowed down run
        sim.startSimulation();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if simulation execution was slowed down
        assertTrue(referenceRuntime * 2.25 < runtime);
        assertTrue(runtime < referenceRuntime * 3.75);
    }

    /**
     * The slow down factor should have an effect on the runtime of an asynchronous simulation
     */
    @Test
    public void setSlowDownFactorAsync() {
        // Set up reference run
        Duration simulationDuration = Duration.ofMillis(5000);
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.registerSimulationObject(new SlowExecutable());

        // Remember reference start time
        long referenceStartTime = System.currentTimeMillis();

        // Start reference rum
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        // Calculate reference runtime
        long referenceRuntime = System.currentTimeMillis() - referenceStartTime;

        // Set up slowed down run
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.setSlowDownFactor(3);
        sim.registerSimulationObject(new SlowExecutable());

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start slowed down run
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if simulation execution was slowed down
        assertTrue(referenceRuntime * 2.25 < runtime);
        assertTrue(runtime < referenceRuntime * 3.75);
    }

    /**
     * The slow down factor to wall clock time should have an effect on the runtime of a synchronous simulation
     */
    @Test
    public void setSlowDownWallClockSync() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(1000);

        // Set up slowed down run
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSlowDownWallClockFactor(3);

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start slowed down run
        sim.startSimulation();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if simulation execution was slowed down
        assertTrue(simulationDuration.toMillis() * 2.25 < runtime);
        assertTrue(runtime < simulationDuration.toMillis() * 3.75);
    }

    /**
     * The slow down factor to wall clock time should have an effect on the runtime of an asynchronous simulation
     */
    @Test
    public void setSlowDownWallClockAsync() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(1000);

        // Set up slowed down run
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.setSlowDownWallClockFactor(3);

        // Remember start time
        long startTime = System.currentTimeMillis();

        // Start slowed down run
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        // Calculate runtime
        long runtime = System.currentTimeMillis() - startTime;

        // Check if simulation execution was slowed down
        assertTrue(simulationDuration.toMillis() * 2.25 < runtime);
        assertTrue(runtime < simulationDuration.toMillis() * 3.75);
    }

    /**
     * The set start daytime should be set and advanced during the simulation
     */
    @Test
    public void setStartDaytime() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        Calendar calendar = Calendar.getInstance();
        calendar.set(2000, Calendar.JANUARY, 1, 0, 0, 0);
        Date startDaytime = calendar.getTime();
        calendar.add(Calendar.MILLISECOND, (int) simulationDuration.toMillis());
        Date expectedDaytimeLower = calendar.getTime();
        calendar.add(Calendar.MILLISECOND, 33);
        Date expectedDaytimeUpper = calendar.getTime();

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setStartDaytime(startDaytime);
        sim.startSimulation();

        // Check if start daytime was set and advanced
        assertTrue(expectedDaytimeLower.before(sim.getDaytime()));
        assertTrue(sim.getDaytime().before(expectedDaytimeUpper));
    }

    /**
     * The set daytime speed up should speed up the simulation day time
     */
    @Test
    public void setDaytimeSpeedUp() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        int daytimeSpeedUp = 60;
        Calendar calendar = Calendar.getInstance();
        calendar.set(2000, Calendar.JANUARY, 1, 0, 0, 0);
        Date startDaytime = calendar.getTime();
        calendar.add(Calendar.MILLISECOND, (int) simulationDuration.toMillis() * daytimeSpeedUp);
        Date expectedDaytimeLower = calendar.getTime();
        calendar.add(Calendar.MILLISECOND, 33 * daytimeSpeedUp);
        Date expectedDaytimeUpper = calendar.getTime();

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setStartDaytime(startDaytime);
        sim.setDaytimeSpeedUp(daytimeSpeedUp);
        sim.startSimulation();

        // Check if start daytime was set and advanced
        assertTrue(expectedDaytimeLower.before(sim.getDaytime()));
        assertTrue(sim.getDaytime().before(expectedDaytimeUpper));
    }

    /**
     * A synchronous simulation should pause at the set pause time
     */
    @Test
    public void setSimulationPauseTimeSync() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        long simulationPauseTime = 500;

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationPauseTime(simulationPauseTime);
        sim.startSimulation();

        Instant expectedTime = Instant.ofEpochMilli(simulationPauseTime);
        // Check if simulation paused correctly
        checkSimTime(sim, expectedTime);
    }

    /**
     * A asynchronous simulation should pause at the set pause time and stay paused
     */
    @Test
    public void setSimulationPauseTimeAsync() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        long simulationPauseTime = 500;

        // Set up and start simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationPauseTime(simulationPauseTime);
        sim.startSimulation();

        // Wait until simulation pauses
        while (!sim.isPaused()) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }

        Instant expectedTime = Instant.ofEpochMilli(simulationPauseTime);
        // Check if simulation reached the set pause time and is paused
        assertTrue(sim.isPaused());
        checkSimTime(sim, expectedTime);

        // Wait
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        // Check if simulation stayed pause
        assertTrue(sim.isPaused());
        checkSimTime(sim, expectedTime);
    }

    /**
     * Tests if error objects are correctly returned
     * Tests if a present error is correctly identified
     * Tests if an occurred error is correctly identified
     * Tests if errorOccurred is correctly reset
     */
    @Test
    public void errorObjectsTest() {
        Duration simulationDuration = Duration.ofMillis(3000);
        Duration continuesSimulationDuration = Duration.ofMillis(500);
        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);

        // Create and register error objects
        TestPhysicalObject someError = new TestPhysicalObject();
        TestPhysicalObject noError = new TestPhysicalObject();
        sim.registerAndPutObject(someError, 10.0, 10.0, 0.0);
        sim.registerAndPutObject(noError, 20.0, 20.0, 0.0);

        // Run simulation
        sim.setSimulationPauseTime(0);
        sim.startSimulation();

        // Check if no error occurred
        assertFalse(sim.errorOccurred());
        // Check if no error is present
        assertFalse(sim.errorPresent());
        // Check if no error objects are returned
        assertTrue(sim.getErrorObjects().isEmpty());

        // Run simulation
        sim.continueSimulation(continuesSimulationDuration);

        // Set computational error
        someError.setError(true);

        // Run simulation
        sim.continueSimulation(continuesSimulationDuration);

        // Check if error occurred
        assertTrue(sim.errorOccurred());
        // Check if error is present
        assertTrue(sim.errorPresent());
        // Check if only the object with error is returned
        assertTrue(sim.getErrorObjects().contains(someError));
        assertFalse(sim.getErrorObjects().contains(noError));

        // Reset error occurred
        sim.resetErrorOccurred();

        // Run simulation
        sim.continueSimulation(continuesSimulationDuration);

        // Check if error occurred
        assertTrue(sim.errorOccurred());
        // Check if error is present
        assertTrue(sim.errorPresent());
        // Check if only the object with error is returned
        assertTrue(sim.getErrorObjects().contains(someError));
        assertFalse(sim.getErrorObjects().contains(noError));

        // Run simulation
        sim.continueSimulation(continuesSimulationDuration);

        // Reset computational error
        someError.setError(false);

        // Run simulation
        sim.continueSimulation(continuesSimulationDuration);

        // Check if error occurred
        assertTrue(sim.errorOccurred());
        // Check if no error is present
        assertFalse(sim.errorPresent());
        // Check if no error objects are returned
        assertTrue(sim.getErrorObjects().isEmpty());

        // Reset error occurred
        sim.resetErrorOccurred();

        // Run simulation
        sim.continueSimulation();

        // Check if no error occurred
        assertFalse(sim.errorOccurred());
        // Check if no error is present
        assertFalse(sim.errorPresent());
        // Check if no error objects are returned
        assertTrue(sim.getErrorObjects().isEmpty());
    }

    /**
     * Tests if observers are correctly registered
     * Tests if observers are correctly unregistered
     * Tests if observers are correctly notified
     */
    @Test
    public void observerTest() {
        Duration simulationDuration = Duration.ofMillis(1000);
        // Set up simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);

        // Create observers
        TestObserver notNotified = new TestObserver();
        TestObserver firstSecondNotified = new TestObserver();
        TestObserver lastSecondNotified = new TestObserver();
        TestObserver alwaysNotified = new TestObserver();

        // Register first, second and fourth observer
        sim.registerLoopObserver(notNotified);
        sim.registerLoopObserver(firstSecondNotified);
        sim.registerLoopObserver(alwaysNotified);

        // Remove first observer and store iteration count
        sim.unregisterLoopObserver(notNotified);
        long iterationCount1 = sim.getLoopCount();

        // Run simulation
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Remove second observer, Register third observer and store iteration count
        sim.unregisterLoopObserver(firstSecondNotified);
        sim.registerLoopObserver(lastSecondNotified);
        long iterationCount2 = sim.getLoopCount();

        // Run simulation
        sim.continueSimulation();

        // Remove third and fourth observer and store iteration count
        sim.unregisterLoopObserver(lastSecondNotified);
        sim.unregisterLoopObserver(alwaysNotified);
        long iterationCount3 = sim.getLoopCount();

        // Check if number of observer calls are correct
        assertEquals(0, notNotified.startCounter);
        assertEquals(0, notNotified.stopCounter);
        assertEquals(iterationCount1, notNotified.didExecCounter);
        assertEquals(iterationCount1, notNotified.willExecCounter);

        assertEquals(1, firstSecondNotified.startCounter);
        assertEquals(0, firstSecondNotified.stopCounter);
        assertEquals(iterationCount2, firstSecondNotified.didExecCounter);
        assertEquals(iterationCount2, firstSecondNotified.willExecCounter);

        assertEquals(0, lastSecondNotified.startCounter);
        assertEquals(1, lastSecondNotified.stopCounter);
        assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.didExecCounter);
        assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.willExecCounter);

        assertEquals(1, alwaysNotified.startCounter);
        assertEquals(1, alwaysNotified.stopCounter);
        assertEquals(iterationCount3, alwaysNotified.didExecCounter);
        assertEquals(iterationCount3, alwaysNotified.willExecCounter);
    }

    /**
     * Tests if simulation objects are correctly registered
     * Tests if simulation objects are correctly unregistered
     * Tests if simulation objects are correctly notified
     */
    @Test
    public void simulationObjectTest() {
        Duration simulationDuration = Duration.ofMillis(1000);
        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);

        // Create simulation objects
        TestExecutable notNotified = new TestExecutable();
        TestExecutable firstSecondNotified = new TestExecutable();
        TestExecutable lastSecondNotified = new TestExecutable();
        TestExecutable alwaysNotified = new TestExecutable();

        // Register first, second and fourth simulation object
        sim.registerSimulationObject(notNotified);
        sim.registerSimulationObject(firstSecondNotified);
        sim.registerSimulationObject(alwaysNotified);

        // Remove first simulation and store iteration count
        sim.unregisterSimulationObject(notNotified);
        long iterationCount1 = sim.getLoopCount();

        // Run simulation
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Remove second simulation object, Register third simulation object and store iteration count
        sim.unregisterSimulationObject(firstSecondNotified);
        sim.registerSimulationObject(lastSecondNotified);
        long iterationCount2 = sim.getLoopCount();

        // Run simulation
        sim.continueSimulation();

        // Remove third and fourth simulation and store iteration count
        sim.unregisterSimulationObject(lastSecondNotified);
        sim.unregisterSimulationObject(alwaysNotified);
        long iterationCount3 = sim.getLoopCount();

        // Check if number of simulation object calls are correct
        assertEquals(iterationCount1, notNotified.execCounter);

        assertEquals(iterationCount2, firstSecondNotified.execCounter);

        assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.execCounter);

        assertEquals(iterationCount3, alwaysNotified.execCounter);
    }

    /**
     * Tests if physical objects are correctly registered
     * Tests if physical objects are correctly unregistered
     * Tests if physical objects are correctly notified
     */
    @Test
    public void physicalObjectTest() {
        Duration simulationDuration = Duration.ofMillis(1000);
        // Set up simulator
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationDuration(simulationDuration);

        // Create physical objects
        TestPhysicalObject notNotified = new TestPhysicalObject();
        TestPhysicalObject firstSecondNotified = new TestPhysicalObject();
        TestPhysicalObject lastSecondNotified = new TestPhysicalObject();
        TestPhysicalObject alwaysNotified = new TestPhysicalObject();

        // Register first, second and fourth physical object
        sim.registerAndPutObject(notNotified, 0.0, 0.0, 0.0);
        sim.registerAndPutObject(firstSecondNotified, 1.0, 1.0, 0.0);
        sim.registerAndPutObject(alwaysNotified, 3.0, 3.0, 0.0);

        // Remove first observer and store iteration count
        sim.unregisterPhysicalObject(notNotified);
        long iterationCount1 = sim.getLoopCount();

        // Run simulation
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Remove second physical object, Register third physical object and store iteration count
        sim.unregisterPhysicalObject(firstSecondNotified);
        sim.registerAndPutObject(lastSecondNotified, 2.0, 2.0, 0.0);
        long iterationCount2 = sim.getLoopCount();

        // Run simulation
        sim.continueSimulation();

        // Remove third and fourth physical and store iteration count
        sim.unregisterPhysicalObject(lastSecondNotified);
        sim.unregisterPhysicalObject(alwaysNotified);
        long iterationCount3 = sim.getLoopCount();

        // Check if number of physical object calls are correct
        assertEquals(iterationCount1, notNotified.computePhysicsCounter);
        assertEquals(1, notNotified.putOnSurfaceCounter);
        assertEquals(iterationCount1, notNotified.executeCounter);

        assertEquals(iterationCount2, firstSecondNotified.computePhysicsCounter);
        assertEquals(1, notNotified.putOnSurfaceCounter);
        assertEquals(iterationCount2, firstSecondNotified.executeCounter);

        assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.computePhysicsCounter);
        assertEquals(1, notNotified.putOnSurfaceCounter);
        assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.executeCounter);

        assertEquals(iterationCount3, alwaysNotified.computePhysicsCounter);
        assertEquals(1, notNotified.putOnSurfaceCounter);
        assertEquals(iterationCount3, alwaysNotified.executeCounter);
    }

    /**
     * The simulation is notifying waiting threads after the specified time
     */
    @Test
    public void waitFor() {
        Duration simulationDuration = Duration.ofMillis(5000);
        // Calculate values
        int simulationLoopFrequency = 30;
        long expectedIterationTime = (long) ((1.0 / simulationLoopFrequency) * 1000);
        Duration expectedWaitForTime = Duration.ofMillis(500);

        // Set up and run simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.setSimulationLoopFrequency(simulationLoopFrequency);
        sim.startSimulation();

        // Remember waiting begin time
        Instant waitBeginTime = sim.getSimulationTime();

        // Wait for the specified time
        sim.waitFor(expectedWaitForTime);

        // Calculated waiting end time
        Duration waitForTime = Duration.between(waitBeginTime, sim.getSimulationTime());

        // Check if thread was notified at the correct time
        assertTrue(expectedWaitForTime.toNanos() <= waitForTime.toNanos());
        // assertTrue(waitForTime < expectedWaitForTime + expectedIterationTime*4);
    }

    /**
     * The simulation is notifying waiting threads at the end of the simulation
     * if the notifying time exceeds the simulation duration
     */
    @Test
    public void waitForEndedToSoon() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);
        Duration waitForTime = simulationDuration.plusMillis(200);

        // Set up and run simulation
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.startSimulation();

        // Wait
        sim.waitFor(waitForTime);

        Instant expectedTime = Instant.ofEpochMilli(simulationDuration.toMillis());
        // Check if thread was notified at the end of the simulation
        checkSimTime(sim, expectedTime);
        // Check if simulation has stopped
        assertFalse(sim.isRunning());
    }

    /**
     * The simulation is notifying waiting threads at the end of the simulation
     */
    @Test
    public void waitUntilSimulationStopped() {
        // Calculate expected values
        Duration simulationDuration = Duration.ofMillis(5000);

        // Set up, run simulation and wait
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationDuration(simulationDuration);
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        Instant expectedTime = Instant.ofEpochMilli(simulationDuration.toMillis());
        // Check if thread was notified at the end of the simulation
        checkSimTime(sim, expectedTime);
        // Check if simulation has stopped
        assertFalse(sim.isRunning());
    }

    private void checkSimTime(Simulator sim, Instant expectedTime) {
        long tolerance = (long) ((1.0 / sim.getSimulationLoopFrequency()) * 1000);
        assertTrue(!expectedTime.isAfter(sim.getSimulationTime()) && sim.getSimulationTime().isBefore(expectedTime.plusMillis(tolerance)));
    }

    private class StepSizeChecker extends SimulationLoopNotifiable {
        private Duration expectedStepSize;

        public StepSizeChecker(Duration expectedStepSize) {
            this.expectedStepSize = expectedStepSize;
        }

        public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {
        }

        @Override
        public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime,
                                    Duration deltaTime) {
            assertEquals(expectedStepSize, Simulator.getSharedInstance().getLastStepSize());
        }

        @Override
        public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime,
                                   Duration deltaTime) {
            assertEquals(expectedStepSize, Simulator.getSharedInstance().getLastStepSize());

        }

        @Override
        public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime,
                                             Duration deltaTime) {
            assertEquals(expectedStepSize, Simulator.getSharedInstance().getLastStepSize());

        }

        @Override
        public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime,
                                            Duration deltaTime) {
            assertEquals(expectedStepSize, Simulator.getSharedInstance().getLastStepSize());

        }

        @Override
        public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {
        }
    }

    private class TestObserver extends SimulationLoopNotifiable {
        private long startCounter = 0;
        private long stopCounter = 0;
        private long willExecCounter = 0;
        private long didExecCounter = 0;

        public TestObserver() {
        }

        @Override
        public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {
            startCounter++;
        }

        @Override
        public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, Instant totalTime) {
            stopCounter++;

        }

        @Override
        public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime,
                                    Duration deltaTime) {
            willExecCounter++;

        }

        @Override
        public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, Instant totalTime,
                                   Duration deltaTime) {
            didExecCounter++;

        }

        @Override
        public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime,
                                             Duration deltaTime) {
        }

        @Override
        public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, Instant totalTime,
                                            Duration deltaTime) {
        }

    }

    private class TestExecutable implements SimulationLoopExecutable {
        private long execCounter = 0;

        public TestExecutable() {
        }

        @Override
        public void executeLoopIteration(Duration timeDiff) {
            execCounter++;

        }
    }

    private class SlowExecutable implements SimulationLoopExecutable {
        private long execCounter = 0;

        public SlowExecutable() {
        }

        @Override
        public void executeLoopIteration(Duration timeDiff) {
            try {
                Thread.sleep(5);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            execCounter++;
        }
    }

    private class TestPhysicalObject implements SimulationLoopExecutable, PhysicalObject {
        private boolean error = false;
        private long uniqueId = IdGenerator.getSharedInstance().generateUniqueId();
        private long computePhysicsCounter = 0;
        private long putOnSurfaceCounter = 0;
        private long executeCounter = 0;

        public TestPhysicalObject() {
        }

        public Vec3 getPosition() {
            return new Vec3(3);
        }

        public void setPosition(Vec3 position) {
        }

        public RealMatrix getRotation() {
            return MatrixUtils.createRealIdentityMatrix(3);
        }

        public void setRotation(RealMatrix rotation) {
        }

        public Vec3 getVelocity() {
            return new Vec3(3);
        }

        public void setVelocity(Vec3 velocity) {
        }

        public Vec3 getAngularVelocity() {
            return new Vec3(3);
        }

        public void setAngularVelocity(Vec3 angularVelocity) {
        }

        public void addForce(Vec3 force) {
        }

        public void addTorque(Vec3 torque) {
        }

        public double getMass() {
            return 0.0;
        }

        public void setMass(double mass) {
        }

        public double getWidth() {
            return 0.0;
        }

        public void setWidth(double width) {
        }

        public double getLength() {
            return 0.0;
        }

        public void setLength(double length) {
        }

        public double getHeight() {
            return 0.0;
        }

        public void setHeight(double height) {
        }

        public Vec3 getGeometryPosition() {
            return new Vec3(3);
        }

        public void setGeometryPosition(Vec3 geometryPosition) {
        }

        public Vec3 getGeometryPositionOffset() {
            return new Vec3(3);
        }

        public void setGeometryPositionOffset(Vec3 geometryPositionOffset) {
        }

        public PhysicalObjectType getPhysicalObjectType() {
            return PhysicalObjectType.PHYSICAL_OBJECT_TYPE_HOUSE;
        }

        public boolean getError() {
            return this.error;
        }

        public void setError(boolean error) {
            this.error = error;
        }

        public boolean getCollision() {
            return false;
        }

        public void setCollision(boolean collision) {
        }

        public long getId() {
            return uniqueId;
        }

        @Deprecated
        public List<Map.Entry<Vec3, Vec3>> getBoundaryVectors() {
            //TODO: Function is unnecessary with three dimensional collision detection
            return new ArrayList<>();
        }

        @Override
        public void computePhysics(Duration deltaTime) {
            computePhysicsCounter++;

        }

        public void putOnSurface(double posX, double posY, double rotZ) {
            putOnSurfaceCounter++;
        }

        @Override
        public void executeLoopIteration(Duration timeDiff) {
            executeCounter++;

        }
    }
}
