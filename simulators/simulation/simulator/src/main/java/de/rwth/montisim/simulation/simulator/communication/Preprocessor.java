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
    float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength);

    /**
     * Calculates the preprocessed output's vehicle state length.
     * @param inputStateLength The input state length, that the preprocessor received.
     * @param inputStatePacketLength The input state packet length, that the preprocessor received.
     * @return The length of the vehicle state length in the preprocessed output array.
     */
    int getStateLength(int inputStateLength, int inputStatePacketLength);

    /**
     * Calculates the preprocessed output's state packet length.
     * @param inputStateLength The input state length, that the preprocessor received.
     * @param inputStatePacketLength The input state packet length, that the preprocessor received.
     * @return The length of each vehicle's state packet length in the preprocessed output array.
     */
    int getStatePacketLength(int inputStateLength, int inputStatePacketLength);

    /**
     * Reshapes an output state of the preprocessor into an array containing the vehicleState and the otherStates.
     * This can be used as input for another preprocessor.
     * @param state The output state of the preprocessor.
     * @param inputStateLength The input state length, that the preprocessor received.
     * @param inputStatePacketLength The input state packet length, that the preprocessor received.
     * @return A float[][][]. The first index points to a float[][] that contains the vehicle state at index 0.
     * The second index points to the other vehicles' state packets.
     */
    default float[][][] reshapeIntoInput(float[] state, int inputStateLength, int inputStatePacketLength) {
        // Init vars
        int stateLength = getStateLength(inputStateLength, inputStatePacketLength);
        int statePacketLength = getStatePacketLength(inputStateLength, inputStatePacketLength);
        int numbOfStatePackets = (state.length - stateLength) / statePacketLength;

        float[][][] result = new float[2][][];
        float[] vehState = new float[stateLength];
        float[][] statePackets = new float[numbOfStatePackets][];

        result[0] = new float[][]{vehState};
        result[1] = statePackets;

        // Fill in the state
        for (int i = 0; i < stateLength; i++) {
            vehState[i] = state[i];
        }

        // Fill in the statePackets
        for (int i = 0; i < numbOfStatePackets; i++) {
            statePackets[i] = new float[statePacketLength];
            for (int j = 0; j < statePacketLength; j++) {
                statePackets[i][j] = state[stateLength + i * statePacketLength + j];
            }
        }

        return result;
    }

    /**
     * Returns the vehicle state from the output state of the preprocessor.
     * @param state The output state of the preprocessor.
     * @param inputStateLength The input state length, that the preprocessor received.
     * @param inputStatePacketLength The input state packet length, that the preprocessor received.
     * @return A float[] containing the vehicleState
     */
    default float[] getVehicleState(float[] state, int inputStateLength, int inputStatePacketLength) {
        return reshapeIntoInput(state, inputStateLength, inputStatePacketLength)[0][0];
    }

    /**
     * Returns the vehicle state packets from the output state of the preprocessor.
     * @param state The output state of the preprocessor.
     * @param inputStateLength The input state length, that the preprocessor received.
     * @param inputStatePacketLength The input state packet length, that the preprocessor received.
     * @return A float[][] containing the vehicles' state packets.
     */
    default float[][] getStatePackets(float[] state, int inputStateLength, int inputStatePacketLength) {
        return reshapeIntoInput(state, inputStateLength, inputStatePacketLength)[1];
    }

}
