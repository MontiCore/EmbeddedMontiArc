/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

/// Object that can be added to the simulation
public interface SimulationObject {
    /// Implementations of this function must register their sub-components by the
    /// simulator: Static/DynamicObject, Updatable, Destroyable
    public void registerComponents(ISimulator simulator);
}