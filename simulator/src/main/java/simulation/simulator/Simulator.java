/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.IdGenerator;
import simulation.util.*;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopNotifiable;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.SimulationLoopExecutable;
import simulation.vehicle.*;
import de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation.PhysicalObject;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.atomic.AtomicLong;
import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * Logic and object management of the simulation
 */
public class Simulator {

    /** Shared instance of the simulator */
    private static Simulator sharedInstance = new Simulator();


    /** Type of the simulation as described in SimulationType class. Default: real-time simulation */
    private SimulationType simulationType = SimulationType.SIMULATION_TYPE_FIXED_TIME;

    /** Whether or not the simulation should not be executed in its own thread and therefore blocking. Default: blocking */
    // TODO: Simulation should always be synchronous
    private boolean synchronousSimulation = true;

    /** Time after which to stop simulation. Default: Infinite */
    private long simulationDuration = Long.MAX_VALUE;

    /** Update frequency of the simulation loop. Default 30 */
    private int simulationLoopFrequency = 30;

    /** Factor by which the simulation is slowed down in fixed_time mode. Default: no slowing down */
    private int slowDownFactor = 1;

    /** Factor by which the simulation is slowed down in fixed_time mode. Default: no slowing down */
    private int slowDownWallClockFactor = 1;

    /** Simulated daytime at the start of the simulation. Default: daytime at creation of class */
    private Date daytimeStart = new Date();

    /** Factor by which the simulated day advances faster than the simulation time. Default: no acceleration */
    private int daytimeSpeedUp = 1;


    /** Simulation time */
    private long simulationTime = 0;

    /** Simulation time at which the simulation will be paused. Default: Never */
    private final AtomicLong simulationPauseTime = new AtomicLong(Long.MAX_VALUE);

    /** Simulation time when the loop was last executed */
    private long simulationTimeLastLoop = 0;

    /** Size of last iteration step */
    private long lastStepSize = 0;

    /** System time at which simulation started */
    private long simulationStartTime = 0;

    /** System time at which the last loop iteration started */
    private long lastLoopStartTime = 0;

    /** Number of loop iterations up to now */
    private long loopCount = 0;

    /** Simulated daytime */
    private Calendar daytime = Calendar.getInstance();

    /** True iff simulation currently running */
    private boolean isRunning = false;

    /** True iff computation of the simulation is currently paused */
    private boolean isPaused = false;

    /** True if a computational error occurred, may be reset by user */
    private boolean errorOccurred = false;

    /** Service calling the simulation loop during real time simulation */
    private Timer timer = null;

    /** All objects that want to be informed about loop executions */
    private final List<SimulationLoopNotifiable> loopObservers = Collections.synchronizedList(new LinkedList<SimulationLoopNotifiable>());

    /** All simulation objects in the simulation */
    private final List<SimulationLoopExecutable> simulationObjects = Collections.synchronizedList(new LinkedList<SimulationLoopExecutable>());

    /** All physical objects in the simulation */
    private final List<PhysicalObject> physicalObjects = Collections.synchronizedList(new LinkedList<PhysicalObject>());

    /** Times for which others wait using the waitFor() method */
    private final List<Long> waitTimers = Collections.synchronizedList(new LinkedList<Long>());


    /**
     * Simulator constructor. Should not be called directly but only by the initialization of "sharedInstance".
     */
    protected Simulator() {
        InformationService.getSharedInstance().offerInformation(Information.SIMULATION_TIME, this::getSimulationTime);
    }

    /**
     * Resets the shared instance of the simulator.
     * Should only be used for testing purposes.
     */
    public static void resetSimulator() {
        IdGenerator.resetInstance();
        sharedInstance = new Simulator();
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

        // Check for sane configuration
        if (simulationType != SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownFactor != 1) {
            throw new IllegalStateException("Only a simulation with type FIXED_TIME can be slowed down.");
        }

        // Check for sane configuration
        if (simulationType != SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownWallClockFactor != 1) {
            throw new IllegalStateException("Only a simulation with type FIXED_TIME can be slowed down to wall clock.");
        }

        // Check for sane configuration
        if (slowDownFactor != 1 && slowDownWallClockFactor != 1) {
            throw new IllegalStateException("Simulation cannot be slowed down and slowed down to wall clock.");
        }

        // Check whether we are already done
        if (simulationTime >= simulationDuration) {
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

        Log.info("Simulation started.");

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

        if (synchronousSimulation) {
            runSimulation();
        } else {
            new Thread(this::runSimulation).start();
        }
    }


    /**
     * This method allows to pause the computation of the simulation after a specified amount
     * of simulated time. For example, if the simulator already simulated 5000 ms (i.e. getSimulationTime()
     * returns 5000), and this method is called with 3000 as parameter then the simulator will pause the
     * computation of the simulation after roughly 8000 ms. To continue the computations either call this
     * method again with a specified amount of time or call continueSimulation() to continue the
     * computation without a new limit. Use isPaused() to check whether or not the simulator is
     * currently pausing its computations.
     * @param timeToPause Simulated time in milliseconds after which the simulator will pause further computations
     */
    public synchronized void continueSimulation(long timeToPause) {
        //Check for sane parameter
        if (timeToPause < 0) {
            throw new IllegalArgumentException("Time to simulation pause " + timeToPause + " should not be negative.");
        }
        if(!isPaused && isRunning){
            throw new IllegalStateException("Simulation cannot be continued when it is not paused.");
        }
        if(!isRunning){
            throw new IllegalStateException("Simulation cannot be continued if it was not started.");
        }
        Log.info("Simulation continued.");
        synchronized (simulationPauseTime) {
            //Set new pause time
            simulationPauseTime.getAndSet(getSimulationTime() + timeToPause);

            if(synchronousSimulation){
                runSimulation();
            }else{
                simulationPauseTime.notifyAll();
            }
        }
    }

    /**
     * Continues the computation of the simulation without a time limit
     */
    public synchronized void continueSimulation() {
        if(!isPaused && isRunning){
            throw new  IllegalStateException("Simulation cannot be continued when it is not paused.");
        }
        if(!isRunning){
            throw new IllegalStateException("Simulation cannot be continued if it was not started.");
        }
        Log.info("Simulation continued.");
        synchronized (simulationPauseTime) {
            //Set new pause time to 'never'
            simulationPauseTime.getAndSet(Long.MAX_VALUE);

            if(synchronousSimulation){
                runSimulation();
            }else{
                simulationPauseTime.notifyAll();
            }
        }
    }
        
    /**
     * Running a simulation. Should only be called in startSimulation()
     */
    private void runSimulation() {
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME ||
                simulationType == SimulationType.SIMULATION_TYPE_MAX_FPS) {

            // Execute simulation
            boolean successfulIteration = true;
            while (successfulIteration) {
                successfulIteration = executeSimulationLoop();
            }

            // Stop simulation if not paused
            if(!isPaused){
                stopSimulation();
            }
        }
    }

    /**
     * Executes one iteration of the Euler simulation loop. Should only by called by simulator objects loopService
     *
     * @return true if the loop was actually executed
     */
    private boolean executeSimulationLoop() {
        // Remember loop start time
        long loopStartTime = System.currentTimeMillis();

        // Advance simulation time
        //TODO: Check if the proposed advancing of the simulation time would overshoot the set pause time or the simulation duration, and then execute a partial step
        switch (simulationType){
            case SIMULATION_TYPE_REAL_TIME:
            case SIMULATION_TYPE_MAX_FPS:
                simulationTime = loopStartTime - simulationStartTime;
                lastStepSize = simulationTime - simulationTimeLastLoop;
                break;
            case SIMULATION_TYPE_FIXED_TIME:
                long timeBetweenCalls = (long) ((1.0 / simulationLoopFrequency) * 1000);
                simulationTime += timeBetweenCalls;
                lastStepSize = timeBetweenCalls;
                break;
        }

        // Inform observers about upcoming loop iteration
        synchronized (loopObservers) {
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.willExecuteLoop(getSimulationObjects(), simulationTime, lastStepSize);
            }
            NotificationCenter.getSharedInstance().postNotification(Notification.NOTIFICATION_LOOP_UPCOMING, null);
        }

        // Loop over simulation objects
        synchronized (simulationObjects) {
            for (SimulationLoopExecutable object : simulationObjects) {

                // Inform observers about upcoming loop iteration for each object
                synchronized (loopObservers) {
                    for (SimulationLoopNotifiable observer : loopObservers) {
                        observer.willExecuteLoopForObject(object, simulationTime, lastStepSize);
                    }
                }

                if(object instanceof PhysicalObject){
                    PhysicsEngine.computePhysics((PhysicalObject) object, getPhysicalObjects(), lastStepSize);
                }

                object.executeLoopIteration(lastStepSize);

                // Inform observers about completed loop iteration for each object
                synchronized (loopObservers) {
                    for (SimulationLoopNotifiable observer : loopObservers) {
                        observer.didExecuteLoopForObject(object, simulationTime, lastStepSize);
                    }
                }
            }
        }

        // Update loop count
        loopCount++;

        // Update loop start time
        lastLoopStartTime = loopStartTime;

        // Update time between last two iterations (i.e. the last iteration and the current one)
        simulationTimeLastLoop = simulationTime;

        // Update simulated daytime
        daytime.add(Calendar.MILLISECOND, (int) lastStepSize * daytimeSpeedUp);

        // Inform observers about completed loop iteration
        synchronized (loopObservers) {
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.didExecuteLoop(getSimulationObjects(), simulationTime, lastStepSize);
            }
            NotificationCenter.getSharedInstance().postNotification(Notification.NOTIFICATION_LOOP_DONE, null);
        }

        //Slow down computations to wall-clock time if requested by user
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownWallClockFactor != 1) {
            long timeDifference = System.currentTimeMillis() - loopStartTime;
            long expectedTimeDifference = (long) ((1.0 / simulationLoopFrequency) * 1000);

            //Only slow down if computation was not already slow enough without slowing it down
            if (timeDifference < expectedTimeDifference * slowDownWallClockFactor) {
                long slowDownTime = expectedTimeDifference * slowDownWallClockFactor  - timeDifference;
                try {
                    Thread.sleep(slowDownTime);
                } catch (InterruptedException e) {
                    Log.warning("Failed to slow down simulation computation." + e);
                }
            }
        }

        //Slow down computation if requested by user
        if (simulationType == SimulationType.SIMULATION_TYPE_FIXED_TIME && slowDownFactor != 1) {
            long timeDifference = System.currentTimeMillis() - loopStartTime;
            try {
                Thread.sleep(timeDifference * (long)(slowDownFactor-1));
            } catch (InterruptedException e) {
                Log.warning("Failed to slow down simulation computation." + e);
            }
        }

        Log.finest("Did loop iteration at simulation time " + simulationTime);

        //See if any thread waited for this simulation time
        synchronized (waitTimers) {
            for (long time : waitTimers) {
                if (simulationTime >= time) {
                    synchronized (sharedInstance){
                        sharedInstance.notifyAll();
                    }
                    break;
                }
            }
            //Clean up list
            waitTimers.removeIf(time -> simulationTime >= time);
        }

        //Remember if computational errors occurred
        if (!errorOccurred) {
            errorOccurred = errorPresent();
        }

        //Check if computation should be paused
        while (simulationTime >= simulationPauseTime.get()) {
            //TODO: With proposed overshoot check, should a equal notify and bigger than throw an error
            isPaused = true;
            Log.info("Simulation paused.");
            if(synchronousSimulation){
                return false;
            }else{
                //Pause computation until unlocked by new time limit
                synchronized (simulationPauseTime) {
                    try {
                        simulationPauseTime.wait();
                    } catch (InterruptedException e) {
                        Log.warning("Thread of simulation with paused computation was interrupted." + e);
                    }
                }
            }
        }

        //We're after the pausing while loop, so we are allowed to continue simulation
        isPaused = false;

        //Check whether we are done
        if (simulationTime >= simulationDuration) {
            //TODO: With proposed overshoot check, should a equal notify and bigger than throw an error
            switch (simulationType){
                case SIMULATION_TYPE_REAL_TIME:
                    stopSimulation();
                    return false;
                case SIMULATION_TYPE_MAX_FPS:
                case SIMULATION_TYPE_FIXED_TIME:
                    return false;
            }
        }
        return true;
    }

    /**
     * Immediately stop the execution of the simulation
     */
    public synchronized void stopSimulation() {
        if(!isRunning){
            throw new IllegalStateException("Simulation is already stopped.");
        }

        Log.info("Simulation stopped after " + simulationTimeLastLoop + " ms. " + loopCount + " simulation loops executed.");

        if (simulationType == SimulationType.SIMULATION_TYPE_REAL_TIME) {
            //Stop calling simulation loop
            timer.cancel();
            timer = null;
        }

        //Set internal state
        isRunning = false;

        // Inform observers about stop of simulation
        synchronized (loopObservers){
            for (SimulationLoopNotifiable observer : loopObservers) {
                observer.simulationStopped(getSimulationObjects(), simulationTime);
            }
        }

        //Wake up waiting threads
        synchronized (sharedInstance) {
            sharedInstance.notifyAll();
        }
    }

    /**
     * Sets the type of the simulation as described in the class SimulationType.
     * May only be set while the simulation is not running.
     *
     * @param type Type of the simulation
     */
    public void setSimulationType(SimulationType type) {
        if (isRunning) {
            throw new IllegalStateException("Cannot change simulation type. Simulation currently running.");
        }
        simulationType = type;
    }

    /**
     * Checks if the simulation is synchronous
     *
     * @return True iff the simulation is synchronous
     */
    public boolean getSynchronousSimulation() {
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
     * Set the simulation duration. Ideally, this value should be set before starting the simulation.
     *
     * @param simulationDuration Time in milliseconds after which the simulation should stop
     */
    public void setSimulationDuration(long simulationDuration) {
        if (simulationDuration < this.simulationTime) {
            throw new IllegalArgumentException("New stop time " + simulationDuration + " should not be in the past.");
        }
        this.simulationDuration = simulationDuration;
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
        simulationDuration += additionalSimulationTime;
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
     * Set the frequency at which the simulation loop should be executed. May be lower in the
     * actual execution. Ask for actual time between last two iterations using
     * getLastStepSize()
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
        if (slowDownWallClockFactor != 1) {
            throw new IllegalStateException("Simulation is already slowed down to wall clock.");
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
            throw new IllegalStateException("Simulation is already slowed down.");
        }
        this.slowDownWallClockFactor = slowDownWallClockFactor;
    }

    /**
     * Sets the daytime at which the simulation should start. May only be set while simulation is not running.
     * @param daytimeStart Daytime at which the simulation starts
     */
    public void setStartDaytime(Date daytimeStart) {
        if (isRunning) {
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
        if (!isPaused && isRunning) {
            throw new IllegalStateException("Cannot update simulation daytime speed up. Simulation currently running.");
        }
        this.daytimeSpeedUp = daytimeSpeedUp;
    }

    /**
     * Get the current simulation time in milliseconds
     *
     * @return Current simulation time in milliseconds
     */
    public long getSimulationTime() {
        return simulationTime;
    }

    /**
     *
     *
     * @param timeToPause
     */
    public synchronized void setSimulationPauseTime(long timeToPause){
        //Check for sane parameter
        if (timeToPause < 0) {
            throw new IllegalArgumentException("Time to simulation pause " + timeToPause + " should not be negative.");
        }
        synchronized (simulationPauseTime) {
            //Set new pause time
            simulationPauseTime.getAndSet(getSimulationTime() + timeToPause);
        }

    }

    /**
     * Return the time between the current loop iteration and the previous loop iteration
     *
     * @return time in ms between last two iterations
     */
    public long getLastStepSize() {
        return lastStepSize;
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
     * Returns the current date in the simulation
     * @return Date giving the current daytime in the simulation
     */
    public Date getDaytime() {
        return daytime.getTime();
    }

    /**
     * Checks if the simulation is currently running
     *
     * @return true if the simulation is currently running
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Returns whether the computation of the simulation is currently paused
     * @return True iff the computation is currently paused
     */
    public synchronized boolean isPaused() {
        return isPaused;
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
        return errorOccurred;
    }

    /**
     * Resets the computational error occurred flag used by errorOccurred(). If the computational error
     * is still present, errorOccurred() will return true after the next simulation
     * loop iteration.
     */
    public void resetErrorOccurred() {
        if (!isPaused && isRunning) {
            throw new IllegalStateException("Cannot reset errorOccurred. Simulation currently running.");
        }
        errorOccurred = false;
    }

    /**
     * Checks whether a computational error is currently present in the simulation
     *
     * @return True iff there is at least one physical objects with a computational error
     */
    public boolean errorPresent() {
        return !getErrorObjects().isEmpty();
    }

    /**
     * Retrieves a list of all objects that are currently to have a computational error.
     *
     * @return List of objects with a computational error
     */
    public List<PhysicalObject> getErrorObjects() {
        LinkedList<PhysicalObject> errorObjects = new LinkedList<>(physicalObjects);
        errorObjects.removeIf(physicalObject -> (!physicalObject.getError()));
        return errorObjects;
    }

    /**
     * Retrieves a copy of the loop observers managed by the simulator
     *
     * @return Copy of the loop observers
     */
    public List<SimulationLoopNotifiable> getLoopObservers() {
        return new LinkedList<>(loopObservers);
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
     * Retrieves a copy of the simulation objects managed by the simulator
     *
     * @return Copy of the simulation objects
     */
    public List<SimulationLoopExecutable> getSimulationObjects() {
        return new LinkedList<>(simulationObjects);
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
     * Retrieves a copy of the physical objects managed by the simulator
     *
     * @return Copy of the physical objects
     */
    public List<PhysicalObject> getPhysicalObjects() {
        return new LinkedList<>(physicalObjects);
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
     * Blocks the calling thread for a given amount of time
     *
     * @param milliseconds Milliseconds of simulation time for which the caller will be blocked
     */
    public synchronized void waitFor(long milliseconds) {
        if (synchronousSimulation) {
            throw new UnsupportedOperationException("Waiting is not available in a synchronous simulation.");
        }

        //Save time at which waiting should be ended
        Long endTime = simulationTime + milliseconds;
        synchronized (waitTimers) {
            waitTimers.add(endTime);
        }

        while (simulationTime < endTime && isRunning) {
            //Go to wait state
            try {
                sharedInstance.wait();
            } catch (InterruptedException e) {
                Log.warning("Could not block thread." + e);
            }
        }
    }

    /**
     * Blocks the calling thread until simulation is finished
     */
    public synchronized void waitUntilSimulationStopped() {
        if (synchronousSimulation) {
            throw new UnsupportedOperationException("Waiting is not available in a synchronous simulation.");
        }

        while (isRunning) {
            //Go to wait state
            try {
                sharedInstance.wait();
            } catch (InterruptedException e) {
                Log.warning("Could not block thread." + e);
            }
        }
    }
}
