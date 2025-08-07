/**
 * (c) https://github.com/MontiCore/monticore
 * <p>
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package simulation.simulator;

/**
 * Logic and object management of the simulation
 */
public class Simulator {

    /** Simulation time  milliseconds at which the simulation will be paused. Default: Never */
    private final AtomicLong simulationPauseTime = new AtomicLong(Long.MAX_VALUE);

    /** Number of loop iterations up to now */
    private long loopCount = 0;

    /** All objects that want to be informed about loop executions */
    private final List<SimulationLoopNotifiable> loopObservers = Collections.synchronizedList(new LinkedList<SimulationLoopNotifiable>());

    /** Times for which others wait using the waitFor() method */
    private final List<Instant> waitTimers = Collections.synchronizedList(new LinkedList<Instant>());

}
