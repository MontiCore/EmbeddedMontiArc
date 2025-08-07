package de.rwth.montisim.simulation.simulator.communication;

/**
 * Preprocessor that processes the state by applying a sequence of other preprocessors.
 */
public class SequencePreprocessor implements Preprocessor {

    Preprocessor[] components;

    /**
     * Initializes the sequence preprocessor.
     * @param components The sequence of preprocessors that should be applied.
     */
    public SequencePreprocessor(Preprocessor[] components) {
        this.components = components;
    }

    @Override
    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        float[] result = new DefaultPreprocessor().preprocessState(vehicleState, otherStates, statePacketLength);
        int stateLength = vehicleState.length;
        for (int i = 0; i < components.length; i++) {
            result = components[i].preprocessState(vehicleState, otherStates, statePacketLength);
            int newStatePacketLength = components[i].getStatePacketLength(stateLength, statePacketLength);
            int newStateLength = components[i].getStateLength(stateLength, statePacketLength);
            vehicleState = components[i].getVehicleState(result, stateLength, statePacketLength);
            otherStates = components[i].getStatePackets(result, stateLength, statePacketLength);
            stateLength = newStateLength;
            statePacketLength = newStatePacketLength;
        }
        return result;
    }

    @Override
    public int getStateLength(int inputStateLength, int inputStatePacketLength) {
        int stateLength = inputStateLength;
        int statePacketLength = inputStatePacketLength;
        for (int i = 0; i < components.length; i++) {
            int newStateLength = components[i].getStateLength(stateLength, statePacketLength);
            int newStatePacketLength = components[i].getStatePacketLength(stateLength, statePacketLength);
            stateLength = newStateLength;
            statePacketLength = newStatePacketLength;
        }
        return stateLength;
    }

    @Override
    public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
        int stateLength = inputStateLength;
        int statePacketLength = inputStatePacketLength;
        for (int i = 0; i < components.length; i++) {
            int newStateLength = components[i].getStateLength(stateLength, statePacketLength);
            int newStatePacketLength = components[i].getStatePacketLength(stateLength, statePacketLength);
            stateLength = newStateLength;
            statePacketLength = newStatePacketLength;
        }
        return statePacketLength;
    }
}
