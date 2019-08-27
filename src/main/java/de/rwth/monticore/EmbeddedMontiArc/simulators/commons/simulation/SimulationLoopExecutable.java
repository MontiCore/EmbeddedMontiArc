/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.monticore.EmbeddedMontiArc.simulators.commons.simulation;

/**
 * Should be implemented by all objects represented in the simulation that should
 * update their state during the simulation. To be updated, objects need to register
 * themselves to the shared instance of the simulator using registerSimulationObject().
 */
public interface SimulationLoopExecutable {
    /**
     * Function that requests the called object to update its state for given time difference
     * @param timeDiffMs Difference in time measured in milliseconds
     */
    void executeLoopIteration(long timeDiffMs);
}
