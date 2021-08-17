package de.rwth.montisim.commons.simulation;

/**
 * Interface for objects of the simulation that require an explicit call
 * when being popped from the simulation, that is deserialized and removed
 * for insertion in another simulator.
 */
public interface Poppable {
    void pop();
}
