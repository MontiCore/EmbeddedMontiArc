/* (c) https://github.com/MontiCore/monticore */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

import java.util.List;

/**
 * Informs objects about state changes in the simulation. To be informed about
 * the simulation loop, you first need to register the object to the shared
 * instance of the simulator using registerLoopObserver().
 * Override the required functions to get notified for a given simulation event.
 */
public abstract class SimulationLoopNotifiable {
    /**
     * Is called just before the simulation objects update their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void willExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    /**
     * Is called after the simulation objects updated their states.
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void didExecuteLoop(List<SimulationLoopExecutable> simulationObjects, long totalTime, long deltaTime) {}

    /**
     * Is called for each object just before the simulation objects update their states.
     *
     * @param simulationObject Object for which the loop will be executed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void willExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    /**
     * Is called for each object after the simulation objects updated their states.
     *
     * @param simulationObject Object for which the loop iteration has been completed
     * @param totalTime Total simulation time in milliseconds
     * @param deltaTime Delta simulation time in milliseconds
     */
    public void didExecuteLoopForObject(SimulationLoopExecutable simulationObject, long totalTime, long deltaTime) {}

    /**
     * Is called just before the simulation starts
     *
     * @param simulationObjects List of all simulation objects
     */
    public void simulationStarted(List<SimulationLoopExecutable> simulationObjects) {}

    /**
     * Is called just after the simulation ends
     *
     * @param simulationObjects List of all simulation objects
     * @param totalTime Total simulation time in milliseconds
     */
    public void simulationStopped(List<SimulationLoopExecutable> simulationObjects, long totalTime) {}
}
