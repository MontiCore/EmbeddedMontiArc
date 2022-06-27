package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.Vec2;

import java.util.Arrays;

/**
 * Preprocessing strategy that only filters out all other vehicles' states except for the closest N vehicles.
 * If there are less than N vehicles, their remaining states are filled with zeros.
 * Additionally, the indicator flag can be used to check whether a state spot is used or not.
 * This preprocessor returns a state array of constant size.
 * The size of the state without the indicator is: vehicleState + statePacketLength * maxNumberOfVehicles
 * The size of the state with the indicator is: vehicleState + (statePacketLength + 1) * maxNumberOfVehicles
 */
public class ProximityFilter implements Preprocessor {

    private final int maxNumberOfVehicles;
    private final boolean addIndicator;

    // constants
    private final int X_POS_STATE_INDEX = 21;
    private final int Y_POS_STATE_INDEX = 22;
    private final int X_POS_PACKET_INDEX = 21;
    private final int Y_POS_PACKET_INDEX = 22;

    /**
     * Initializes the ProximityFilter.
     * @param maxNumberOfVehicles The number of vehicle states that are permitted.
     * @param addIndicator Whether an indicator should be added per state packet that is set to 1 if
     *                     a packet is available and 0 if not.
     */
    public ProximityFilter(int maxNumberOfVehicles, boolean addIndicator) {
        this.maxNumberOfVehicles = maxNumberOfVehicles;
        this.addIndicator = addIndicator;
    }
    @Override
    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        // If there are no state packets, just return the original state
        if (statePacketLength == 0) return vehicleState;

        // Otherwise init variables
        if (addIndicator) statePacketLength++;
        int stateLength = vehicleState.length;
        float[] result = new float[stateLength+ maxNumberOfVehicles * statePacketLength];

        // Add the first state
        for (int i = 0; i < stateLength; i++) result[i] = vehicleState[i];

        // Create an index array
        Integer[] index = new Integer[otherStates.length];
        for (int i = 0; i < index.length; i++) index[i] = i;

        // Sort the indices by their distance to the vehicle
        Vec2 pos1 = new Vec2(vehicleState[X_POS_STATE_INDEX], vehicleState[Y_POS_STATE_INDEX]);
        Arrays.sort(index, (a, b) -> {
            double distA = pos1.distance(new Vec2(otherStates[a][X_POS_PACKET_INDEX], otherStates[a][Y_POS_PACKET_INDEX]));
            double distB = pos1.distance(new Vec2(otherStates[b][X_POS_PACKET_INDEX], otherStates[b][Y_POS_PACKET_INDEX]));
            if (distA < distB) return -1;
            else if (distA > distB) return 1;
            else return 0;
        });

        // Add first N states, starting with the closest vehicles
        for (int i = 0; i < Math.min(index.length, maxNumberOfVehicles); i++) {
            for (int j = 0; j < statePacketLength; j++) {
                if (addIndicator && j == statePacketLength - 1) {
                    result[stateLength + i * statePacketLength + j] = 1;
                } else {
                    result[stateLength + i * statePacketLength + j] = otherStates[index[i]][j];
                }
            }
        }

        // Fill in the remaining state with zeros
        for (int i = 0; i < maxNumberOfVehicles - index.length; i++) {
            for (int j = 0; j < statePacketLength; j++) {
                result[stateLength + statePacketLength * (index.length + i) + j] = 0;
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
        if (addIndicator) {
            return inputStatePacketLength + 1;
        } else {
            return inputStatePacketLength;
        }
    }
}
