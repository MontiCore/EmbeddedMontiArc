package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

/**
 * This Preprocessor converts each angle value of a vehicle state into a sine and a cosine value.
 */
public class AnglePreprocessor implements Preprocessor {
    private final int angleStateIndex;
    private final int angleStatePacketIndex;

    public AnglePreprocessor(int angleStateIndex, int angleStatePacketIndex) {
        this.angleStateIndex = angleStateIndex;
        this.angleStatePacketIndex = angleStatePacketIndex;
    }

    @Override
    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        // Init array
        float[] result = new float[vehicleState.length + 1 + otherStates.length * (statePacketLength + 1)];
        int stateLength = vehicleState.length;
        // Append vehicle state
        boolean addOne = false;
        for (int i = 0; i < stateLength; i++) {
            if (i == angleStateIndex) {
                // Convert the angle into a sin and a cos state
                float sin = (float) Math.sin(vehicleState[i] / 180 * Math.PI);
                float cos = (float) Math.cos(vehicleState[i] / 180 * Math.PI);
                result[i] = sin;
                result[i + 1] = cos;
                addOne = true;
            } else {
                result[addOne? i + 1 : i] = vehicleState[i];
            }
        }
        // Append state packets
        for (int i = 0; i < otherStates.length; i++) {
            addOne = false;
            for (int j = 0; j < statePacketLength; j++) {
                if (j == angleStatePacketIndex) {
                    // Convert the angle into a sin and a cos state
                    float sin = (float) Math.sin(otherStates[i][j] / 180 * Math.PI);
                    float cos = (float) Math.cos(otherStates[i][j] / 180 * Math.PI);
                    result[stateLength + 1 + i * (statePacketLength + 1) + j] = sin;
                    result[stateLength + 1 + i * (statePacketLength + 1) + j + 1] = cos;
                    addOne = true;
                } else {
                    result[stateLength + 1 + i * (statePacketLength + 1) + j + (addOne? 1 : 0)] = otherStates[i][j];
                }
            }
        }
        return result;
    }

    @Override
    public int getStateLength(int inputStateLength, int inputStatePacketLength) {
        return inputStateLength + 1;
    }

    @Override
    public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
        return inputStatePacketLength + 1;
    }
}
