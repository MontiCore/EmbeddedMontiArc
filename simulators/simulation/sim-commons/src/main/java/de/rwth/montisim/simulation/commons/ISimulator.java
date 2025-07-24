/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.simulation.commons;

import de.rwth.montisim.commons.simulation.*;

/**
 * Interface for registering different SimulationObject components by the simulator.
 */
public interface ISimulator {

    void registerStaticObject(SimulationObject obj, StaticObject staticObject);

    /// If registering a dynamic object, do not register it as static
    void registerDynamicObject(SimulationObject obj, DynamicObject dynObject);

    void registerUpdatable(SimulationObject obj, Updatable updatable);

    void registerDestroyable(SimulationObject obj, Destroyable destroyable);

    void registerTaskRunner(SimulationObject obj, TaskRunner runner);
}