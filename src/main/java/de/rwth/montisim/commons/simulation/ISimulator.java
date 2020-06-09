/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

/**
 * Interface for registering different SimulationObject components by the simulator.
 */
public interface ISimulator {

    void registerStaticObject(StaticObject staticObject);

    /// If registering a dynamic object, do not register it as static
    void registerDynamicObject(DynamicObject dynObject);

    void registerUpdatable(Updatable updatable);

    void registerDestroyable(Destroyable destroyable);

    void registerTaskRunner(TaskRunner runner);
}