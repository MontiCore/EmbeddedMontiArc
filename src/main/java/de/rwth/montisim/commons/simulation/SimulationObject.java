/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

/**
 * Object that can be added to the simulation.
 * 
 * The simulator must call 'registerComponents()' on the object when it is added to the simulation.
 * This allows the SimulationObject to register its different components by the Simulator.
 */
public interface SimulationObject {
    /**
     * Implementations of this function must register their sub-components by the
     * simulator: Static/DynamicObject, Updatable, Destroyable
     */
    public void registerComponents(ISimulator simulator);
}