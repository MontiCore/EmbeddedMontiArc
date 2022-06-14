package de.rwth.montisim.simulation.simulator.communication;

/**
 * Interface that allows preprocessing the states of vehicles before sending them to the agent.
 */
public interface Preprocessor {
    /**
     * Preprocesses the state arrays of all vehicles into a combined state.
     * @param vehicleState float[] containing the currently active vehicle's state.
     * @param otherStates float[][] where each float[] contains the state packet of a different vehicle.
     * @return A float[] containing the preprocessed combined state of the vehicles.
     * @throws Exception When the states could not be preprocessed.
     */
    float[] preprocessState(float[] vehicleState, float[][] otherStates);

}
