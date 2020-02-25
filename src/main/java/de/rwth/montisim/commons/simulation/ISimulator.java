/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

public interface ISimulator {

    void registerStaticObject(StaticObject staticObject);

    /// If registering a dynamic object, do not register it as static
    void registerDynamicObject(DynamicObject dynObject);

    void registerUpdatable(Updatable updatable);

    void registerDestroyable(Destroyable destroyable);
}