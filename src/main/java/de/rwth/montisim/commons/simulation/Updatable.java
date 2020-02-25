/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.rwth.montisim.commons.simulation;

import java.time.Duration;

/**
 * Interface for updatable objects in the simulation. Gets called for every tick
 * update. If the object has child Updatable Objects, it must propagate the
 * update call.
 */
public interface Updatable {
    /**
     * Function that requests the called object to update its state for given time
     * difference.
     * 
     * @param deltaT Time difference between last and current state.
     */
    void update(Duration deltaT);
}
