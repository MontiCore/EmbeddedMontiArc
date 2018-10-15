package simulation.simulator;

import commons.simulation.IdGenerator;
import simulation.util.*;
import commons.simulation.SimulationLoopNotifiable;
import commons.simulation.SimulationLoopExecutable;
import simulation.vehicle.*;
import commons.simulation.PhysicalObject;
import java.util.*;
import java.util.concurrent.atomic.AtomicLong;

/**
 * Logic and object management of the simulation
 */
public class Simulator {

    /** Shared instance of the simulator */
    private static Simulator sharedInstance = new Simulator();

    /** Simulation parameters with default values */
    /** Update frequency of the simulation loop. Default 30 */
    private int simulationLoopFrequency = 30;

    /** Type of the simulation as described in SimulationType class. Default: real-time simulation */
    private SimulationType simulationType = SimulationType.SIMULATION_TYPE_FIXED_TIME;

    /**
     * Factor by which the simulation is slowed down in fixed_time mode. See setSlowDownFactor() for
     * more information. Default: no slowing down
     */
    private int slowDownFactor = 1;

    /**
     * Factor by which the simulation is slowed down in fixed_time mode. See
     * setSlowDownWallClockFactor() for more information. Default: no slowing down
     */
    private int slowDownWallClockFactor = 0;

    /** Simulated daytime at the start of the simulation. Default: daytime at creation of class */
    private Date daytimeStart = new Date();

    /** Factor by which the simulated day advances faster than the simulation time. Default: no acceleration */
    private int daytimeSpeedUp = 1;

    /** Whether or not the simulation should not be executed in its own thread and therefore blocking. Default: blocking */
    private boolean synchronousSimulation = true;

    /** Whether the simulation will be paused or extended in the future */
    private boolean isPausedInFuture = false;


    /** Simulation time */
    private long simulationTime = 0;

    /** Simulation time when the loop was last executed */
    private long lastLoopTime = 0;

    /** Time between last two iteration of the loop */
    private long timeBetweenLastIterations = 0;

    /** Service calling the simulation loop */
    private Timer timer = null;

    /** System time at which simulation started */
    private long simulationStartTime = 0;

    /** Time after which to stop simulation. Default: Infinite */
    private long simulationStopTime = Long.MAX_VALUE;

    /** Simulation time at which the simulation will be paused. Default: Never */
    private final AtomicLong simulationPauseTime = new AtomicLong(Long.MAX_VALUE);

    /** System time at which the last loop iteration started */
    private long lastLoopStartTime = 0;

    /** Simulated daytime */
    private Calendar daytime = Calendar.getInstance();

    /** Number of loop iterations up to now */
    private long loopCount = 0;

    /** True iff simulation currently running */
    private boolean isRunning = false;

    /** True iff computation of the simulation is currently paused */
    private boolean isComputationPaused = false;

    /** True if a computational error occurred, may be reset by user */
    private boolean errorOccurredDuringExecution = false;

    /** All objects that want to be informed about loop executions */
    private final List<SimulationLoopNotifiable> loopObservers = Collections.synchronizedList(new LinkedList<SimulationLoopNotifiable>());

    /** All simulation objects in the simulation */
    private final List<SimulationLoopExecutable> simulationObjects = Collections.synchronizedList(new LinkedList<SimulationLoopExecutable>());

    /** All physical objects in the simulation */
    private final List<PhysicalObject> physicalObjects = Collections.synchronizedList(new LinkedList<PhysicalObject>());

    /** Times for which others wait using the waitForTime() method */
    private final List<Long> waitTimers = Collections.synchronizedList(new LinkedList<Long>());

    /**
     * Resets the shared instance of the simulator.
     * Should only be used for testing purposes.
     */
    public static void resetSimulator() {
        IdGenerator.resetInstance();
        sharedInstance = new Simulator();
    }

    /**
     * Simulator constructor. Should not be called directly but only by the initialization of "sharedInstance".
     */
    protected Simulator() {
        InformationService.getSharedInstance().offerInformation(Information.SIMULATION_TIME, this::getSimulationTime);
    }

    /**
     * Starts the simulation if possible. If the simulation can not be started, a log message is printed.
     * Use isCurrentlyRunning() to check whether starting the simulation was successful.
     * Use resetSimulator() before starting a new simulation. Otherwise, the old simulation will be
     * continued.
     */
    public void startSimulation() {
        // Check for sane frequency
        if (simulationLoopFrequency <= 0) {
            throw  new IllegalStateException("Simulation loop frequency " + simulationLoopFrequency + " is not positive.");
        }

        // Check simulation is not already running
        if (isRunning) {
            throw new IllegalStateException("Simulation is already running.");
        }

        // Check for sane configuration
        if (synchronousSimulation && simulationType == SimulationType.SIMULATION_TYPE_REAL_TIME) {
            throw new IllegalStateException("A simulation with type REAL_TIME cannot be executed synchronously.");
        }

        // Check for sane configurytion
        if (simulationType != SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownFactor != 1) {
            throw new IllegalStateException("Only a simulation with type FIXED_TIME can be slowed down.");
        }

        //Check whether we are already done
        if (simulationTime >= simulationStopTime) {
            return;
        }

        //Reset time
        simulationStartTime = System.currentTimeMillis();

        //Set simulated daytime
        daytime.setTime(daytimeStart);

        //Set internal state
        isRunning = true;

        // Inform observers about upcoming simulation start
        synchronized (loopObservers) {
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.simulationStarted(getSimulationObjects());
            }
        }

        if (simulationType == SimulationType.SIMULATION_TYPE_REAL_TIME) {
            //Schedule loop calls
            TimerTask loopIteration = new TimerTask() {
                @Override
                public void run() {
                    Simulator.getSharedInstance().executeSimulationLoop();
                }
            };
            //Convert simulation loop frequency to milliseconds between loop calls
            long timeBetweenCalls = (long) ((1.0 / simulationLoopFrequency) * 1000);

            timer = new Timer();
            timer.scheduleAtFixedRate(loopIteration, 0, timeBetweenCalls);
        }

        Log.info("Simulation " + ((loopCount == 0) ? "started." : "continued."));

        if (synchronousSimulation) {
            runSimulation();
        } else {
            new Thread(this::runSimulation).start();
        }
    }
        
    /**
     * Running a simulation. Should only be called in startSimulation()
     */
    private void runSimulation() {
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME ||
                simulationType == SimulationType.SIMULATION_TYPE_MAX_FPS) {

            //Execute simulation
            boolean successfulIteration = true;
            while (successfulIteration) {
                successfulIteration = executeSimulationLoop();
            }
        }
    }

    /**
     * Immediately stop the execution of the simulation
     */
    public synchronized void stopSimulation() {
        if (simulationType == SimulationType.SIMULATION_TYPE_REAL_TIME) {
            //Stop calling simulation loop
            timer.cancel();
            timer = null;
        }

        //Set internal state
        isRunning = false;

        // Inform observers about simulation stop if it is not a pause
        if (!isPausedInFuture) {
            for (SimulationLoopNotifiable observer : this.loopObservers) {
                observer.simulationStopped(getSimulationObjects(), simulationTime);
            }
        }

        //Keep current daytime for future time extensions
        daytimeStart = getDaytime();

        // Count objects with a computational error
        int errorCount = getErrorObjects().size();

        //Inform user
        Log.info("Simulation " + (isPausedInFuture ? "paused" : "stopped") + " after " + lastLoopTime + " ms. " + loopCount + " simulation loops executed. Number of physical objects with a computational error: " + errorCount);

        //Wake up waiting threads
        sharedInstance.wakeupWaitingThreads();
    }

    /**
     * Set the simulation duration. Ideally, this value should be set before starting the simulation.
     *
     * @param simulationStopTime Time in milliseconds after which the simulation should stop
     */
    public void stopAfter(long simulationStopTime) {
        if (simulationStopTime < this.simulationTime) {
            throw new IllegalArgumentException("New stop time " + simulationStopTime + " should not be in the past.");
        }
        this.simulationStopTime = simulationStopTime;
    }

    /**
     * Extend the simulation time.
     *
     * @param additionalSimulationTime Time in ms that the simulation should be extended by. Must be positive.
     */
    public void extendSimulationTime(long additionalSimulationTime) {
        if (additionalSimulationTime < 0) {
            throw new IllegalArgumentException("Additional simulation time " + additionalSimulationTime + " should not be negative");
        }
        simulationStopTime += additionalSimulationTime;
    }

    /**
     * Executes one iteration of the Euler simulation loop. Should only by called by simulator objects loopService
     *
     * @return true if the loop was actually executed
     */
    private boolean executeSimulationLoop() {
        if (simulationType == SimulationType.SIMULATION_TYPE_REAL_TIME ||
                simulationType == SimulationType.SIMULATION_TYPE_MAX_FPS) {
            //Update time
            simulationTime = System.currentTimeMillis() - simulationStartTime;
        } else if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME) {
            //Convert simulation loop frequency to milliseconds between loop calls
            long timeBetweenCalls = (long) ((1.0 / simulationLoopFrequency) * 1000);

            simulationTime += timeBetweenCalls;
        }

        //Check whether we are done
        if (simulationTime >= simulationStopTime) {
            simulationTime = lastLoopTime;
            stopSimulation();
            return false;
        }

        //Slow down computations to wall-clock time if requested by user
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownWallClockFactor != 0) {
            long timeDifference = System.currentTimeMillis() - lastLoopStartTime;
            long expectedTimeDifference = (long) ((1.0 / simulationLoopFrequency) * 1000);

            //Only slow down if computation was not already slow enough without slowing it down
            if (timeDifference < expectedTimeDifference * slowDownWallClockFactor) {
                long slowDownTime = expectedTimeDifference * slowDownWallClockFactor - timeDifference;
                try {
                    Thread.sleep(slowDownTime);
                } catch (InterruptedException e) {
                    Log.warning("Failed to slow down simulation computation." + e);
                }
            }
        }

        //Check if computation should be paused
        while (simulationTime >= simulationPauseTime.get()) {
            //Pause computation until unlocked by new time limit
            synchronized (simulationPauseTime) {
                try {
                    isComputationPaused = true;
                    simulationPauseTime.wait();
                } catch (InterruptedException e) {
                    Log.warning("Thread of simulation with paused computation was interrupted." + e);
                }
            }
        }

        //We're after the pausing while loop, so we are allowed to continue simulation
        isComputationPaused = false;

        //Remember computation start time
        long loopStartTime = System.currentTimeMillis();
        lastLoopStartTime = loopStartTime;

        //Update time between last two iterations (i.e. the last iteration and the current one)
        timeBetweenLastIterations = simulationTime - lastLoopTime;
        lastLoopTime = simulationTime;

        //Update simulated daytime
        daytime.add(Calendar.MILLISECOND, (int)timeBetweenLastIterations * daytimeSpeedUp);

        synchronized (loopObservers) {
            //Inform observers about upcoming loop iteration
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.willExecuteLoop(getSimulationObjects(), simulationTime, timeBetweenLastIterations);
            }

            NotificationCenter.getSharedInstance().postNotification(Notification.NOTIFICATION_LOOP_UPCOMING, null);
        }

        synchronized (simulationObjects) {
            for (SimulationLoopExecutable object : simulationObjects) {

                // Inform observers about upcoming loop iteration for each object
                synchronized (loopObservers) {
                    for (SimulationLoopNotifiable observer : loopObservers) {
                        observer.willExecuteLoopForObject(object, simulationTime, timeBetweenLastIterations);
                    }
                }

                //Execute loop
                if(object instanceof PhysicalObject){
                    PhysicsEngine.computePhysics((PhysicalObject) object, getPhysicalObjects(), timeBetweenLastIterations);
                }

                object.executeLoopIteration(timeBetweenLastIterations);

                // Inform observers about completed loop iteration for each object
                synchronized (loopObservers) {
                    for (SimulationLoopNotifiable observer : loopObservers) {
                        observer.didExecuteLoopForObject(object, simulationTime, timeBetweenLastIterations);
                    }
                }
            }
        }

        synchronized (loopObservers) {
            //Inform observers about completed loop iteration
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.didExecuteLoop(getSimulationObjects(), simulationTime, timeBetweenLastIterations);
            }

            NotificationCenter.getSharedInstance().postNotification(Notification.NOTIFICATION_LOOP_DONE, null);
        }

        synchronized (waitTimers) {
            //See if any thread waited for this simulation time
            for (long time : waitTimers) {
                if (simulationTime >= time) {
                    wakeupWaitingThreads();
                    break;
                }
            }

            //Clean up list
            waitTimers.removeIf(time -> simulationTime >= time);
        }

        //Remember if computational errors occurred. Only check this if we don't
        //already know about computational errors to save processing power
        if (!errorOccurredDuringExecution) {
            errorOccurredDuringExecution = errorPresent();
        }

        Log.finest("Did loop iteration at simulation time " + simulationTime);
        loopCount++;

        //Slow down computation if requested by user
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownFactor != 1) {
            long timeDifference = System.currentTimeMillis() - loopStartTime;
            try {
                Thread.sleep(timeDifference * (long)(slowDownFactor-1));
            } catch (InterruptedException e) {
                Log.warning("Failed to slow down simulation computation." + e);
            }
        }

        return true;
    }


    /**
     * Blocks the calling thread until simulation is finished
     */
    public synchronized void waitUntilSimulationFinished() {
        if (synchronousSimulation) {
            throw new UnsupportedOperationException("Waiting is not available in a synchronous simulation.");
        }

        while (isSimulationRunning()) {
            //Go to wait state
            try {
                sharedInstance.wait();
            } catch (InterruptedException e) {
                Log.warning("Could not block thread." + e);
            }
        }
    }

    /**
     * Blocks the calling thread for a given amount of time
     *
     * @param milliseconds Milliseconds of simulation time for which the caller will be blocked
     */
    public synchronized void waitForTime(long milliseconds) {
        if (synchronousSimulation) {
            throw new UnsupportedOperationException("Waiting is not available in a synchronous simulation.");
        }

        //Save time at which waiting should be ended
        Long endTime = simulationTime + milliseconds;
        synchronized (waitTimers) {
            waitTimers.add(endTime);
        }

        while (simulationTime < endTime) {
            //Go to wait state
            try {
                sharedInstance.wait();
            } catch (InterruptedException e) {
                Log.warning("Could not block thread." + e);
            }
        }
    }

    private synchronized void wakeupWaitingThreads() {
        sharedInstance.notifyAll();
    }

    /**
     * Register an object to receive information about the loop execution
     *
     * @param observer the object to be informed
     */
    public void registerLoopObserver(SimulationLoopNotifiable observer) {
        synchronized (loopObservers) {
            if (!loopObservers.contains(observer)) {
                loopObservers.add(observer);
            }
        }
    }

    /**
     * Unregister an object to not longer receive information about the loop execution
     *
     * @param observer The observer to be removed
     */
    public void unregisterLoopObserver(SimulationLoopNotifiable observer) {
        synchronized (loopObservers) {
            loopObservers.remove(observer);
        }
    }

    /**
     * Registers a simulation object to the simulation. But only if it is not also a physical object.
     * Register physical objects with registerAndPutObject().
     *
     * @param executable The simulation object to be updated during loop iterations
     */
    public void registerSimulationObject(SimulationLoopExecutable executable) {
        synchronized (simulationObjects) {
            if(executable instanceof PhysicalObject){
                throw new IllegalArgumentException("Physical object " + executable + "should be registered using registerAndPutObject().");
            }
            if (!simulationObjects.contains(executable)) {
                simulationObjects.add(executable);
            }
        }
    }

    /**
     * Removes a simulation object from the simulation. But only if it is not also a physical object.
     * Remove physical objects with unregisterPhysicalObject().
     *
     * @param executable The simulation object to no longer be updated during loop iterations
     */
    public void unregisterSimulationObject(SimulationLoopExecutable executable) {
        synchronized (simulationObjects) {
            if(executable instanceof PhysicalObject){
                throw new IllegalArgumentException("Physical object " + executable + "should be unregistered using unregisterPhysicalObject().");
            }
            simulationObjects.remove(executable);
        }
    }

    /**
     * Adds a physical object to the simulation. If the physical object is also a simulation object
     * than is it is also registered as a simulation object.
     *
     * @param object The physical object to be updated during loop iterations
     */
    private void registerPhysicalObject(PhysicalObject object) {
        synchronized (physicalObjects) {
            if (!physicalObjects.contains(object)) {
                physicalObjects.add(object);
            }

            if(object instanceof SimulationLoopExecutable){
                SimulationLoopExecutable executable = (SimulationLoopExecutable) object;
                synchronized (simulationObjects){
                    if (!simulationObjects.contains(executable)) {
                        simulationObjects.add(executable);
                    }
                }
            }
        }
    }

    /**
     * Removes a physical object from the simulation. If the physical object is also a simulation object
     * than is it is also removed as a simulation object.
     *
     * @param object The physical object to no longer be updated during loop iterations
     */
    public void unregisterPhysicalObject(PhysicalObject object) {
        synchronized (physicalObjects) {
            physicalObjects.remove(object);
            if(object instanceof SimulationLoopExecutable){
                synchronized (simulationObjects){
                    SimulationLoopExecutable executable = (SimulationLoopExecutable) object;
                    simulationObjects.remove(executable);
                }
            }
        }
    }

    /**
     * Adds a physical object to the simulation at a specified position and with a specified rotation.
     * If the physical object is also a simulation object
     * than is it is also registered as a simulation object.
     *
     * @param object The physical object to be updated during loop iterations
     * @param posX Position x of the object
     * @param posY Position y of the object
     * @param rotZ Rotation z of the object
     */
    public void registerAndPutObject(PhysicalObject object, double posX, double posY, double rotZ) {
        // Register
        registerPhysicalObject(object);

        // Put object on the surface of the simulation
        object.putOnSurface(posX, posY, rotZ);
    }

    /**
     * Retrieves a copy of the physical objects managed by the simulator
     *
     * @return Copy of the physical objects
     */
    public List<PhysicalObject> getPhysicalObjects() {
        return new LinkedList<>(physicalObjects);
    }

    /**
     * Retrieves a copy of the simulation objects managed by the simulator
     *
     * @return Copy of the simulation objects
     */
    public List<SimulationLoopExecutable> getSimulationObjects() {
        return new LinkedList<>(simulationObjects);
    }

    /**
     * Retrieves a list of all objects that are known to have a computational error.
     *
     * @return List of objects with a computational error
     */
    public List<PhysicalObject> getErrorObjects() {
        LinkedList<PhysicalObject> errorObjects = new LinkedList<>(physicalObjects);
        errorObjects.removeIf(physicalObject -> (!physicalObject.getError()));
        return errorObjects;
    }

    /**
     * Checks whether a computational error is currently present in the simulation
     *
     * @return True iff there is at least one physical objects with a computational error
     */
    public boolean errorPresent() {
        return getErrorObjects().size() > 0;
    }

    /**
     * Checks whether a computational error occurred during the execution of the simulation.
     * This does NOT imply that a computational error is currently present, but only that
     * a computational error happened at some point in the simulation. This information may be
     * reset while the simulation is not running using resetErrorOccurred(). This
     * method together with resetErrorOccurred() can be handy to check if a computational error
     * happened during the last extension of the simulation.
     *
     * @return True iff a computational error occurred since resetErrorOccurred()
     */
    public boolean errorOccurred() {
        return errorOccurredDuringExecution;
    }

    /**
     * Resets the computational error occurred flag used by errorOccurred(). If the computational error
     * is still present, errorOccurred() will return true after the next simulation
     * loop iteration.
     */
    public void resetErrorOccurred() {
        if (isSimulationRunning()) {
            throw new IllegalStateException("Cannot reset errorOccured. Simulation currently running.");
        }
        errorOccurredDuringExecution = false;
    }

    /**
     * Provides access to the shared instance of the Simulator.
     *
     * @return The shared instance of the simulator.
     */
    public static Simulator getSharedInstance() {
        return sharedInstance;
    }

    /**
     * Returns the frequency at which the Euler loop
     *
     * @return true if the simulation is currently running
     */
    public int getSimulationLoopFrequency() {
        return simulationLoopFrequency;
    }

    /**
     * Return the time between the current loop iteration and the previous loop iteration
     *
     * @return time in ms between last two iterations
     */
    public long getTimeBetweenLastIterations() {
        return timeBetweenLastIterations;
    }

    /**
     * Set the frequency at which the simulation loop should be executed. May be lower in the
     * actual execution. Ask for actual time between last two iterations using
     * getTimeBetweenLastIterations()
     *
     * @param simulationLoopFrequency frequency in Hz
     */
    public void setSimulationLoopFrequency(int simulationLoopFrequency) {
        if (simulationLoopFrequency <= 0) {
            throw new IllegalArgumentException("Simulation loop frequency " + simulationLoopFrequency + " must be positive.");
        }
        this.simulationLoopFrequency = simulationLoopFrequency;
    }

    /**
     * Checks if the simulation is currently running
     *
     * @return true if the simulation is currently running
     */
    public boolean isSimulationRunning() {
        return isRunning;
    }

    /**
     * Get the current simulation time in milliseconds
     *
     * @return Current simulation time in milliseconds
     */
    public Long getSimulationTime() {
        return simulationTime;
    }

    /**
     * Sets the daytime at which the simulation should start. May only be set while simulation is not running.
     * @param daytimeStart Daytime at which the simulation starts
     */
    public void setStartDaytime(Date daytimeStart) {
        if(daytimeStart == null){
            throw new IllegalArgumentException("Start day time should not be null.");
        }
        if (isSimulationRunning()) {
            throw new IllegalStateException("Cannot update simulation start daytime. Simulation currently running.");
        }
        this.daytimeStart = daytimeStart;
    }

    /**
     * Sets the daytime speed up. May only be set while simulation is not running.
     * @param daytimeSpeedUp Factor by which the simulated day advances in relation to simulated time
     */
    public void setDaytimeSpeedUp(int daytimeSpeedUp) {
        if(daytimeSpeedUp < 1){
            throw new IllegalArgumentException("Daytime speed up " + daytimeSpeedUp + " should not be less than 1.");
        }
        if (isSimulationRunning()) {
            throw new IllegalStateException("Cannot update simulation daytime speed up. Simulation currently running.");
        }
        this.daytimeSpeedUp = daytimeSpeedUp;
    }

    /**
     * Returns the current date in the simulation
     * @return Date giving the current daytime in the simulation
     */
    public Date getDaytime() {
        return daytime.getTime();
    }

    /**
     * Sets the type of the simulation as described in the class SimulationType.
     * May only be set while the simulation is not running.
     *
     * @param type Type of the simulation
     */
    public void setSimulationType(SimulationType type) {
        if (isSimulationRunning()) {
            throw new IllegalStateException("Cannot change simulation type. Simulation currently running.");
        }
        simulationType = type;
    }

    /**
     * Get number of loop iterations
     *
     * @return Count of loop iterations
     */
    public long getLoopCount() {
        return loopCount;
    }

    /**
     * Checks if the simulation is synchronous
     *
     * @return True iff the simulation is synchronous
     */
    public boolean isSynchronousSimulation() {
        return synchronousSimulation;
    }

    /**
     * Configure whether the simulation should be synchronous, i.e., run its own
     * thread or be blocking. Default: false.
     *
     * @param synchronousSimulation True iff the simulation should be blocking.
     */
    public void setSynchronousSimulation(boolean synchronousSimulation) {
        this.synchronousSimulation = synchronousSimulation;
    }

    /**
     * Checks whether the simulator assumes the simulation will be paused or extended
     * in the future
     *
     * @return True iff the simulator assumes the simulation will be paused or extended
     */
    public boolean getIsPausedInFuture() {
        return isPausedInFuture;
    }

    /**
     * Inform the simulator if the simulation will be paused or extended in the future.
     * This knowledge is used to inform objects waiting for the simulation to be finished
     * only after it is really finished.
     *
     * @param pausedInFuture True iff the simulation will be paused or extended
     */
    public void setIsPausedInFuture(boolean pausedInFuture) {
        isPausedInFuture = pausedInFuture;
    }

    /**
     * A factor by which the simulation computation is slowed down. This does not affect the
     * simulated time between two frames. For example a factor of 2 doubles the wall-clock
     * time needed to run the simulation and a factor of 3 triples the wall-clock time needed
     * to execute the simulation. Default is 1, i.e., no slowing down. This mechanism is only
     * applied if the simulation is run with type SIMULATION_TYPE_FIXED_TIME because slowing
     * down the computation time contradicts the intention of the other types.
     *
     * However, keep in mind that this method only sets a lower bound. The simulation may take
     * more time to be calculated.
     *
     * This method cannot be used simultaneously with setSlowDownWallClockFactor().
     *
     * @param slowDownFactor Factor by which the computations are slowed down.
     */
    public synchronized void setSlowDownFactor(int slowDownFactor) {
        if (slowDownFactor < 1) {
            throw new IllegalArgumentException("New slow down factor " + slowDownFactor + " should not be less than 1.");
        }
        if (slowDownWallClockFactor != 0) {
            Log.warning("setSlowDownFactor() cannot be used simultaneously with setSlowDownWallClockFactor(). Call setSlowDownWallClockFactor(0) before using this method.");
            return;
        }
        this.slowDownFactor = slowDownFactor;
    }

    /**
     * This method slows down the computation of the simulation roughly to wall-clock time.
     * For example, a simulation with 30 frames per second, i.e., 33 ms per frame will take 33 ms
     * of wall-clock time to compute a frame. A factor can be provided to further slow down the computation.
     * The factor will be multiplied by the wall-clock time to calculate the lower bound for the
     * computation time. For example, simulating 6 seconds with factor 2 will take about 12 secs
     * of wall clock time. Default is a factor of 0, i.e., no slowing down.
     *
     * However, keep in mind that this method only sets a rough lower bound. The simulation may take
     * more time to be calculated.
     *
     * This method cannot be used simultaneously with setSlowDownWallClockFactor().
     *
     * @param slowDownWallClockFactor Factor of wall-clock time. Factor*wall-clock time is lower bound for runtime.
     */
    public synchronized void setSlowDownWallClockFactor(int slowDownWallClockFactor) {
        if (slowDownWallClockFactor < 1) {
            throw new IllegalArgumentException("New wall clock slow down factor " + slowDownWallClockFactor + " should not be less than 1.");
        }
        if (slowDownFactor != 1) {
            Log.warning("setSlowDownWallClockFactor() cannot be used simultaneously with setSlowDownFactor(). Call setSlowDownFactor(1) before using this method.");
            return;
        }
        this.slowDownWallClockFactor = slowDownWallClockFactor;
    }

    /**
     * This method allows to pause the computation of the simulation after a specified amount
     * of simulated time. For example, if the simulator already simulated 5000 ms (i.e. getSimulationTime()
     * returns 5000), and this method is called with 3000 as parameter then the simulator will pause the
     * computation of the simulation after roughly 8000 ms. To continue the computations either call this
     * method again with a specified amount of time or call continueComputations() to continue the
     * computation without a new limit. Use isComputationPaused() to check whether or not the simulator is
     * currently pausing its computations.
     * @param timeToPause Simulated time in milliseconds after which the simulator will pause further computations
     */
    public synchronized void pauseComputationsAfter(long timeToPause) {
        //Check for sane parameter
        if (timeToPause < 0) {
            throw new IllegalArgumentException("Time to simulation pause " + timeToPause + " should not be negative.");
        }
        synchronized (simulationPauseTime) {
            //Set new pause time
            simulationPauseTime.getAndSet(getSimulationTime() + timeToPause);

            //Wake up possibly paused computation
            simulationPauseTime.notifyAll();
        }
    }

    /**
     * Returns whether the computation of the simulation is currently paused
     * @return True iff the computation is currently paused
     */
    public synchronized boolean isComputationPaused() {
        return isComputationPaused;
    }

    /**
     * Continues the computation of the simulation without a time limit
     */
    public synchronized void continueComputations() {
        synchronized (simulationPauseTime) {
            //Set new pause time to 'never'
            simulationPauseTime.getAndSet(Long.MAX_VALUE);

            //Wake up possibly paused computation
            simulationPauseTime.notifyAll();
        }
    }
}