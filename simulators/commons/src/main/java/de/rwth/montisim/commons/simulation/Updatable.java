/* (c) https://github.com/MontiCore/monticore */
package de.rwth.montisim.commons.simulation;

/**
 * Interface for updatable objects in the simulation. Gets called for every tick
 * update. If the object has child Updatable Objects, it must propagate the
 * update call.
 */
public interface Updatable {
    void update(TimeUpdate newTime);
}
