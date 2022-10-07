/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons;

/**
 * Object that can be added to the simulation.
 * <p>
 * The simulator must call 'registerComponents()' on the object when it is added to the simulation.
 * This allows the SimulationObject to register its different components by the Simulator.
 */
public abstract class SimulationObject {
    /**
     * Implementations of this function must register their sub-components by the
     * simulator: Static/DynamicObject, Updatable, Destroyable
     */
    public abstract void registerComponents(ISimulator simulator);

    public transient ISimulatorState state;
}