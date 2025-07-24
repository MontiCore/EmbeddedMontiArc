package de.rwth.montisim.simulation.simulator.communication;

import org.jfree.util.Log;

/**
 * Implements the default preprocessing strategy of just combining the states of all vehicles into an array.
 * This results in the state size being dependent on the number of vehicles.
 */
public class DefaultPreprocessor implements Preprocessor {

    final static String STATE_PACKET_LENGTH_ERROR = "Preprocessor received different state packet lengths from different vehicles.";

    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        // If there are no other vehicle states, return just the current vehicle state
        if (statePacketLength == 0) return vehicleState;
        // Check whether the state packets are malformed
        for (int i = 1; i < otherStates.length; i++) {
            if (otherStates[i].length != statePacketLength) {
                Log.error(STATE_PACKET_LENGTH_ERROR);
            }
        }
        // Combine the states
        float[] result = new float[vehicleState.length + otherStates.length * statePacketLength];
        for (int i = 0; i < vehicleState.length; i++) {
            result[i] = vehicleState[i];
        }
        for (int i = 0; i < otherStates.length; i++) {
            for (int j = 0; j < statePacketLength; j++) {
                result[vehicleState.length + i * statePacketLength + j] = otherStates[i][j];
            }
        }

        return result;
    }

    @Override
    public int getStateLength(int inputStateLength, int inputStatePacketLength) {
        return inputStateLength;
    }

    @Override
    public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
        return inputStatePacketLength;
    }

}
