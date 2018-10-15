package simulation.simulator;

import commons.simulation.*;
import org.apache.commons.math3.linear.*;
import org.junit.*;
import simulation.util.*;
import java.util.*;
import static org.junit.Assert.assertTrue;

/**
 * Class that tests the Simulator class
 */
public class SimulatorTest {

    @BeforeClass
    public static void setUpClass() {
        Log.setLogEnabled(false);
    }

    @AfterClass
    public static void tearDownClass() {
        Log.setLogEnabled(true);
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalArgumentException.class)
    public void setSimulationLoopFrequencyFail() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationLoopFrequency(0);
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalStateException.class)
    public void startSimulationSynchronousRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(true);
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.startSimulation();
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSlowDownFactor(2);
        sim.startSimulation();
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownMaxFPS() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_MAX_FPS);
        sim.setSlowDownFactor(2);
        sim.startSimulation();
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownWallClockRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.setSlowDownWallClockFactor(2);
        sim.startSimulation();
    }

    /**
     * Prevent simulation with invalid settings
     */
    @Test(expected = IllegalStateException.class)
    public void startSimulationSlowedDownWallClockMaxFPS() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_MAX_FPS);
        sim.setSlowDownWallClockFactor(2);
        sim.startSimulation();
    }

    /**
     * Tests if observers are correctly registered
     * Tests if observers are correctly unregistered
     * Tests if observers are correctly notified
     */
    @Test
    public void observerTest() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.stopAfter(1000);

        // Create observers
        NotificationCounter notNotified = new NotificationCounter();
        NotificationCounter firstSecondNotified = new NotificationCounter();
        NotificationCounter lastSecondNotified = new NotificationCounter();
        NotificationCounter alwaysNotified = new NotificationCounter();

        // Register first, second and fourth observer
        sim.registerLoopObserver(notNotified);
        sim.registerLoopObserver(firstSecondNotified);
        sim.registerLoopObserver(alwaysNotified);

        // Remove first observer and store iteration count
        sim.unregisterLoopObserver(notNotified);
        long iterationCount1 = sim.getLoopCount();

        // Run simulation for 0.5 seconds
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        // Remove second observer, Register third observer and store iteration count
        sim.unregisterLoopObserver(firstSecondNotified);
        sim.registerLoopObserver(lastSecondNotified);
        long iterationCount2 = sim.getLoopCount();

        // Run simulation for 1 second and then stop
        sim.continueSimulation();

        // Remove third and fourth observer and store iteration count
        sim.unregisterLoopObserver(lastSecondNotified);
        sim.unregisterLoopObserver(alwaysNotified);
        long iterationCount3 = sim.getLoopCount();

        // Check if number of observer calls are correct
        Assert.assertEquals(0, notNotified.startCounter);
        Assert.assertEquals(0, notNotified.stopCounter);
        Assert.assertEquals(iterationCount1, notNotified.didExecCounter);
        Assert.assertEquals(iterationCount1, notNotified.willExecCounter);

        Assert.assertEquals(1, firstSecondNotified.startCounter);
        Assert.assertEquals(0, firstSecondNotified.stopCounter);
        Assert.assertEquals(iterationCount2, firstSecondNotified.didExecCounter);
        Assert.assertEquals(iterationCount2, firstSecondNotified.willExecCounter);

        Assert.assertEquals(0, lastSecondNotified.startCounter);
        Assert.assertEquals(1, lastSecondNotified.stopCounter);
        Assert.assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.didExecCounter);
        Assert.assertEquals(iterationCount3 - iterationCount2, lastSecondNotified.willExecCounter);

        Assert.assertEquals(1, alwaysNotified.startCounter);
        Assert.assertEquals(1, alwaysNotified.stopCounter);
        Assert.assertEquals(iterationCount3, alwaysNotified.didExecCounter);
        Assert.assertEquals(iterationCount3, alwaysNotified.willExecCounter);
    }

    /**
     * Unregistering simulation objects should stop calling the methods of the interface
     */
    @Test
    public void unregisteringPhysicalObjects() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Create dummy objects
        SimObject a = new SimObject();
        SimObject b = new SimObject();

        //(Un)registering physical objects
        sim.registerAndPutObject(a, 10.0, 10.0, 0.0);
        assertTrue(sim.getPhysicalObjects().contains(a));
        assertTrue(!sim.getPhysicalObjects().contains(b));
        sim.registerAndPutObject(b, 20.0, 20.0, 0.0);
        assertTrue(sim.getPhysicalObjects().contains(a));
        assertTrue(sim.getPhysicalObjects().contains(b));

        sim.unregisterPhysicalObject(a);
        assertTrue(!sim.getPhysicalObjects().contains(a));
        assertTrue(sim.getPhysicalObjects().contains(b));
        sim.unregisterPhysicalObject(b);
        assertTrue(!sim.getPhysicalObjects().contains(a));
        assertTrue(!sim.getPhysicalObjects().contains(b));

        //Unregistering physical objects that are registered as simulation objects is legal
        sim.registerAndPutObject(a, 10.0, 10.0, 0.0);
        sim.unregisterPhysicalObject(a);
        assertTrue(!sim.getPhysicalObjects().contains(a));

        //Unregistering simulation objects also removes physical object
        sim.unregisterPhysicalObject(a);
        assertTrue(!sim.getPhysicalObjects().contains(a));
    }

    /**
     * Returning objects with error returns objects with error only
     */
    @Test
    public void returnErrorObjects() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        SimObject a = new SimObject();
        SimObject b = new SimObject();
        SimObject c = new SimObject();

        sim.registerAndPutObject(a, 10.0, 10.0, 0.0);
        sim.registerAndPutObject(b, 20.0, 20.0, 0.0);
        sim.registerAndPutObject(c, 30.0, 30.0, 0.0);

        //No objects with error -> empty list
        List<PhysicalObject> error = sim.getErrorObjects();
        assertTrue(error.size() == 0);

        //error occurred -> return objects with error
        a.setError(true);
        error = sim.getErrorObjects();
        assertTrue(error.contains(a) && !error.contains(b) && !error.contains(c));
        b.setError(true);
        error = sim.getErrorObjects();
        assertTrue(error.contains(a) && error.contains(b) && !error.contains(c));

        //Error resolved -> Not returned anymore
        a.setError(false);
        error = sim.getErrorObjects();
        assertTrue(!error.contains(a) && error.contains(b) && !error.contains(c));
        b.setError(false);
        error = sim.getErrorObjects();
        assertTrue(error.size() == 0);
    }

    /**
     * Checking error memory
     */
    @Test
    public void errorObjectsMemory() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        SimObject a = new SimObject();
        SimObject b = new SimObject();
        SimObject c = new SimObject();

        sim.registerAndPutObject(a, 10.0, 10.0, 0.0);
        sim.registerAndPutObject(b, 20.0, 20.0, 0.0);
        sim.registerAndPutObject(c, 30.0, 30.0, 0.0);

        //No objects with error
        List<PhysicalObject> error = sim.getErrorObjects();
        assertTrue(sim.errorPresent() == false);
        assertTrue(sim.errorOccurred() == false);

        //Error occurred
        a.setError(true);
        sim.stopAfter(100);
        sim.startSimulation();
        assertTrue(sim.errorPresent() == true);
        assertTrue(sim.errorOccurred() == true);

        //Reset computational error flag
        sim.resetErrorOccurred();
        assertTrue(sim.errorPresent() == true);
        assertTrue(sim.errorOccurred() == false);

        //Extending simulation updates memory
        sim.extendSimulationTime(100);
        sim.startSimulation();
        assertTrue(sim.errorPresent() == true);
        assertTrue(sim.errorOccurred() == true);

        //Error resolved
        a.setError(false);
        assertTrue(sim.errorPresent() == false);
        assertTrue(sim.errorOccurred() == true);
    }

    /**
     * Checks that executeLoopIteration is called for every loop iteration
     */
    @Test
    public void executable() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Add a notifiable object
        Executable exec = new Executable();
        sim.registerSimulationObject(exec);

        // Set simulation duration (5 seconds)
        sim.stopAfter(5000);

        //Run simulation
        sim.startSimulation();

        //Function calls should match actual loop iterations
        long frames = sim.getLoopCount();
        assertTrue(frames == exec.execCounter);
    }

    /**
     * Slowing down the simulation actually slows down computation
     */
    @Test
    public void slowDownSynchronousComputation() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        // Set simulation duration (5 seconds)
        sim.stopAfter(5000);

        //Add a notifiable object
        SlowExecutable exec = new SlowExecutable();
        sim.registerSimulationObject(exec);

        //Remember start time of simulation
        long startTime = System.currentTimeMillis();

        sim.startSimulation();

        //Calculate runtime of normal running simulation
        long referenceRuntime = System.currentTimeMillis() - startTime;

        //Second run with slowed down simulation
        sim.setSlowDownFactor(3);
        startTime = System.currentTimeMillis();
        sim.extendSimulationTime(5000);
        sim.startSimulation();

        //Compare runtimes
        long slowedRuntime = System.currentTimeMillis() - startTime;

        assertTrue(referenceRuntime * 2.25 < slowedRuntime);
        assertTrue(referenceRuntime * 3.75 > slowedRuntime);
    }

    /**
     * Slowing down the simulation actually slows down computation
     */
    @Test
    public void slowDownSynchronousComputationToWallClockTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        // Set simulation duration
        sim.stopAfter(1000);

        long startTime = System.currentTimeMillis();

        //Run slowed down simulation
        sim.setSlowDownWallClockFactor(3);
        sim.startSimulation();

        long runtime = System.currentTimeMillis() - startTime;

        //Compare runtime
        assertTrue(runtime >= 2800);

        //Try with different parameters for frequency and slow down factor
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();
        sim.setSimulationLoopFrequency(66);

        // Set simulation duration
        sim.stopAfter(800);

        startTime = System.currentTimeMillis();

        //Run slowed down simulation
        sim.setSlowDownWallClockFactor(5);
        sim.startSimulation();

        runtime = System.currentTimeMillis() - startTime;

        //Compare runtime
        assertTrue(runtime >= 3800);
    }

    /**
     * Slowing down the simulation actually slows down computation
     */
    @Test
    public void slowDownAsynchronousComputation() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);

        // Set simulation duration (5 seconds)
        sim.stopAfter(5000);

        //Add a notifiable object
        SlowExecutable exec = new SlowExecutable();
        sim.registerSimulationObject(exec);

        //Remember start time of simulation
        long startTime = System.currentTimeMillis();

        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        //Calculate runtime of normal running simulation
        long referenceRuntime = System.currentTimeMillis() - startTime;

        //Second run with slowed down simulation
        sim.setSlowDownFactor(3);
        startTime = System.currentTimeMillis();
        sim.extendSimulationTime(5000);
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        //Compare runtimes
        long slowedRuntime = System.currentTimeMillis() - startTime;

        assertTrue(referenceRuntime * 2.25 < slowedRuntime);
        assertTrue(referenceRuntime * 3.75 > slowedRuntime);
    }

    /**
     * Slowing down the simulation actually slows down computation
     */
    @Test
    public void slowDownAsynchronousComputationToWallClockTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);

        // Set simulation duration
        sim.stopAfter(1000);

        long startTime = System.currentTimeMillis();

        //Run slowed down simulation
        sim.setSlowDownWallClockFactor(3);
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        long runtime = System.currentTimeMillis() - startTime;

        //Compare runtime
        assertTrue(runtime >= 2800);

        //Try with different parameters for frequency and slow down factor
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();
        sim.setSimulationLoopFrequency(66);
        sim.setSynchronousSimulation(false);

        // Set simulation duration
        sim.stopAfter(800);

        startTime = System.currentTimeMillis();

        //Run slowed down simulation
        sim.setSlowDownWallClockFactor(5);
        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        runtime = System.currentTimeMillis() - startTime;

        //Compare runtime
        assertTrue(runtime >= 3800);
    }

    /**
     * Test the manual pausing of computations of asynchronous simulation
     */
    @Test
    public void pauseAsyncComputations() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);

        // Set simulation duration
        sim.stopAfter(1000);

        //Run paused simulation
        sim.setSimulationPauseTime(500);
        sim.startSimulation();
        sim.waitFor(500 - sim.getSimulationTime() - 10);

        //Some extra time to enter pausing
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        assertTrue(sim.isComputationPaused());
        long simTime = sim.getSimulationTime();

        //We waited the right amount of time + one frame tolerance
        checkSimTime(sim, 500, 33);

        //We're not advancing in simulation time
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        assertTrue(sim.isComputationPaused());
        assertTrue(sim.getSimulationTime() == simTime);

        //Run to finish
        sim.continueSimulation();
    }

    /**
     * Test the manual pausing of computations of synchronous simulation
     */
    @Test
    public void pauseSyncComputations() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        // Set simulation duration
        sim.stopAfter(1000);

        //Run paused simulation
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        assertTrue(sim.isComputationPaused());
        assertTrue(sim.isRunning());
        long simTime = sim.getSimulationTime();

        //We waited the right amount of time + one frame tolerance
        checkSimTime(sim, 500, 33);

        //Run to finish
        sim.continueSimulation();
    }

    /**
     * The simulation frequency may be set to positive, but not to negative values.
     * Trying to set a negative frequency should not change the current frequency.
     */
    @Test
    public void setSimulationFrequencyNormal() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationLoopFrequency(20);
        Assert.assertEquals(20, sim.getSimulationLoopFrequency());
    }

    /**
     * The desired simulation time should be (more or less) met.
     */
    @Test
    public void stopTime () {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        // Set simulation duration (5 seconds)
        sim.stopAfter(5000);

        //Start simulation
        sim.startSimulation();

        //Stop time should be equal to desired runtime. Tolerance: 1 iteration time = 33ms.
        Long stopTime = sim.getSimulationTime();
        checkSimTime(sim, stopTime, 33);
    }

    /**
     * A simulation that should stop after 0 ms should stop immediately
     */
    @Test
    public void immediatelyStopSimulation() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Immediately stopping simulation
        sim.stopAfter(0);
        sim.startSimulation();

        //Test no simulation time has passed
        Long stopTime = sim.getSimulationTime();
        assertTrue(stopTime == 0);
    }

    /**
     * Continuing a simulation should increase simulation time
     */
    @Test
    public void continueSimulationIncreasesSimTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Start a simulation
        sim.stopAfter(1000);
        sim.startSimulation();

        //Get simulated time
        Long time = sim.getSimulationTime();

        //Continue for the same amount of time as before
        sim.stopAfter(2000);
        sim.startSimulation();
        checkSimTime(sim, 2000, 33);
    }

    /**
     * Running the same simulation twice should lead to same amount of simulated time and frames
     */
    @Test
    public void synchronousSimulationIsDeterministic() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Start a simulation
        sim.stopAfter(1000);
        sim.startSimulation();

        //Get simulated time
        long simTime1 = sim.getSimulationTime();
        long loopCount1 = sim.getLoopCount();

        //Reset simulator
        Simulator.resetSimulator();
        sim = Simulator.getSharedInstance();

        //Start another run
        sim.stopAfter(1000);
        sim.startSimulation();

        //Get simulated time
        long simtTime2= sim.getSimulationTime();
        long loopCount2 = sim.getLoopCount();

        //Compare to first run
        assertTrue(simTime1 == simtTime2);
        assertTrue(loopCount1 == loopCount2);
    }

    /**
     * Test that extendSimulationTime() actually increases simulation time
     */
    @Test
    public void extendSimulationTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Start a simulation
        sim.stopAfter(1000);
        sim.setSimulationPauseTime(500);
        sim.startSimulation();

        //Continue for the same amount of time as before
        sim.extendSimulationTime(1000);
        sim.continueSimulation();
        checkSimTime(sim, 2000, 33);
    }

    /**
     * Test that extendSimulationTime() actually increases simulation time
     */
    @Test(expected = IllegalArgumentException.class)
    public void extendSimulationTimeInvalid() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.extendSimulationTime(-5);
    }

    /**
     * Test the simulated daytime and daytime speedup
     */
    @Test
    public void daytimeSimulation() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        // Set daytime, speedup: 1 second = 1 day
        Calendar cal = Calendar.getInstance();
        cal.set(2000, Calendar.JANUARY, 1, 12, 0, 0);
        Date startTime = cal.getTime();
        sim.setStartDaytime(startTime);
        sim.setDaytimeSpeedUp(86400);

        // Set simulation duration (5 seconds)
        sim.stopAfter(5000);
        sim.startSimulation();

        // Test if 5 days passed. 50 minutes tolerance (~1 frame)
        cal.add(Calendar.DAY_OF_MONTH, 5);
        cal.add(Calendar.MINUTE, -50);
        Date before = cal.getTime();
        cal.add(Calendar.MINUTE, 100);
        Date after = cal.getTime();

        assertTrue(after.after(sim.getDaytime()));
        assertTrue(before.before(sim.getDaytime()));

        // Extend by another 10 seconds
        sim.extendSimulationTime(10000);
        sim.setStartDaytime(startTime);
        sim.setDaytimeSpeedUp(3600);
        sim.startSimulation();

        //Test if 10 hours passed. 2 minutes tolerance (~1 frame)
        cal.set(2000, Calendar.JANUARY, 1, 12, 0, 0);
        cal.add(Calendar.HOUR_OF_DAY, 10);
        cal.add(Calendar.MINUTE, -2);
        before = cal.getTime();
        cal.add(Calendar.MINUTE, 4);
        after = cal.getTime();

        assertTrue(after.after(sim.getDaytime()));
        assertTrue(before.before(sim.getDaytime()));
    }

    /**
     * Ensures getters provide right results
     */
    @Test
    public void getters() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();

        //Synchronous simulation
        sim.setSynchronousSimulation(true);
        assertTrue(sim.isSynchronousSimulation());
        sim.setSynchronousSimulation(false);
        assertTrue(!sim.isSynchronousSimulation());
    }

    /**
     * Fixed time simulation has fixed time
     * //todo check time notivied at all notifications
     */
    @Test
    public void fixedTimeIntervals() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_FIXED_TIME);

        // Calculate expected iteration time
        long expectedIterationTime = (long) ((1.0 / 30) * 1000);

        // Create and register observer that tests the iteration time
        TimeChecker checker = new TimeChecker(expectedIterationTime);
        sim.registerLoopObserver(checker);

        // Run simulation
        sim.stopAfter(5000);
        sim.startSimulation();
    }

    /**
     * Real time simulation actually takes real time
     */
    @Test
    public void realTimeSimTakesRealTime() {
        Simulator.resetSimulator();
        Simulator sim = Simulator.getSharedInstance();
        sim.setSynchronousSimulation(false);
        sim.setSimulationType(SimulationType.SIMULATION_TYPE_REAL_TIME);
        sim.stopAfter(2000);

        long millisBefore = System.currentTimeMillis();

        sim.startSimulation();
        sim.waitUntilSimulationStopped();

        long runtime = System.currentTimeMillis() - millisBefore;

        //Runtime should be within 10% deviance
        //assertTrue(runtime - 2000 < 200);
        assertTrue(runtime >= 2000 - 200);
        assertTrue(sim.getSimulationTime() > 1800);
    }

    private void checkSimTime(Simulator sim, long expectedTime, long tolerance){
         Assert.assertTrue(expectedTime <= sim.getSimulationTime()&& sim.getSimulationTime() < expectedTime + tolerance);
    }
    private class NotificationCounter implements SimulationLoopNotifiable {
        private long startCounter = 0;
        private long stopCounter = 0;
        private long willExecCounter = 0;
        private long didExecCounter = 0;

        public NotificationCounter(){}
        public void simulationStarted(List<SimulationLoopExecutable> simulationObjects){
            startCounter++;
        }
        public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime){
            stopCounter++;
        }
        public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime){}
        public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime){}
        public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime){
            willExecCounter++;
        }
        public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {
            didExecCounter++;
        }
    }

    private class TimeChecker implements SimulationLoopNotifiable {
        private long expectedTime;

        public TimeChecker(long expectedTime){
            this.expectedTime = expectedTime;
        }
        public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}
        public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {}
        public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}
        public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}
        public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}
        public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {
            Assert.assertEquals(expectedTime, Simulator.getSharedInstance().getLastStepSize());
        }
    }

    private class Executable implements SimulationLoopExecutable {
        public long execCounter = 0;

        public Executable(){}
        public void executeLoopIteration(long timeDiffMs) {
            execCounter++;
        }
    }

    private class SlowExecutable implements SimulationLoopExecutable {
        public long execCounter = 0;

        public SlowExecutable(){}
        public void executeLoopIteration(long timeDiffMs) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            execCounter++;
        }
    }

    private class SimObject implements SimulationLoopExecutable, PhysicalObject {
        private boolean error = false;
        private long uniqueId = IdGenerator.getSharedInstance().generateUniqueId();

        public SimObject() {}
        public RealVector getPosition(){
            return new ArrayRealVector(3);
        }
        public void setPosition(RealVector position){}
        public RealMatrix getRotation(){
            return new BlockRealMatrix(3, 3);
        }
        public void setRotation(RealMatrix rotation){}
        public RealVector getVelocity(){
            return new ArrayRealVector(3);
        }
        public void setVelocity(RealVector velocity){}
        public RealVector getAngularVelocity(){
            return new ArrayRealVector(3);
        }
        public void setAngularVelocity(RealVector angularVelocity){}
        public void addForce(RealVector force){}
        public void addTorque(RealVector torque){}
        public double getMass(){
            return 0.0;
        }
        public void setMass(double mass){}
        public double getWidth(){
            return 0.0;
        }
        public void setWidth(double width){}
        public double getLength(){
            return 0.0;
        }
        public void setLength(double length){}
        public double getHeight(){
            return 0.0;
        }
        public void setHeight(double height){}
        public RealVector getGeometryPosition(){
            return new ArrayRealVector(3);
        }
        public void setGeometryPosition(RealVector geometryPosition){}
        public RealVector getGeometryPositionOffset(){
            return new ArrayRealVector(3);
        }
        public void setGeometryPositionOffset(RealVector geometryPositionOffset){}
        public PhysicalObjectType getPhysicalObjectType(){
            return PhysicalObjectType.PHYSICAL_OBJECT_TYPE_HOUSE;
        }
        public boolean getError(){
            return this.error;
        }
        public void setError(boolean error){
            this.error = error;
        }
        public boolean getCollision(){
            return false;
        }
        public void setCollision(boolean collision){}
        public long getId(){
            return uniqueId;
        }
        public List<Map.Entry<RealVector, RealVector>> getBoundaryVectors(){
            // ToDo Function is unnecessary with three dimensional collision detection
            return new ArrayList<>();
        }
        public void computePhysics(long deltaTms){}
        public void putOnSurface(double posX, double posY, double rotZ){}
        public void executeLoopIteration(long timeDiffMs){}
    }
}