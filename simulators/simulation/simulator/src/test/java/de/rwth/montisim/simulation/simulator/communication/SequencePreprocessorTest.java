package de.rwth.montisim.simulation.simulator.communication;

import junit.framework.TestCase;
import org.junit.Assert;

public class SequencePreprocessorTest extends TestCase {

    public void testPreprocessState() {
        // Test state and state packet length
        PreprocessorA proA = new PreprocessorA();
        PreprocessorB proB = new PreprocessorB();
        SequencePreprocessor seq = new SequencePreprocessor(new Preprocessor[]{proA, proB});
        Assert.assertEquals(0, seq.getStateLength(2, 3));
        Assert.assertEquals(7, seq.getStatePacketLength(2, 3));

        float[] state = {1f, 2f, 3f};
        float[][] packets = {{10f, 20f, 30f}, {20f, 30f, 40f}};

        // Test empty sequence
        seq = new SequencePreprocessor(new Preprocessor[0]);
        Assert.assertEquals(2, seq.getStateLength(2, 3));
        Assert.assertEquals(3, seq.getStatePacketLength(2, 3));
        float[] expected = {1f, 2f, 3f, 10f, 20f, 30f, 20f, 30f, 40f};
        Assert.assertArrayEquals(expected, seq.preprocessState(state, packets, packets[0].length), 0.001f);

        // Test with one preprocessor
        seq = new SequencePreprocessor(new Preprocessor[]{proA});
        expected = new float[]{1f, 2f, 3f, 1f, 2f, 3f, 10f, 20f, 30f, 20f, 30f, 40f};
        Assert.assertArrayEquals(expected, seq.preprocessState(state, packets, packets[0].length), 0.001f);

        // Test with two preprocessor
        seq = new SequencePreprocessor(new Preprocessor[]{proA, proB});
        expected = new float[]{10f, 20f, 30f, 1f, 2f, 3f, 1f, 2f, 3f, 20f, 30f, 40f, 1f, 2f, 3f, 1f, 2f, 3f};
        Assert.assertArrayEquals(expected, seq.preprocessState(state, packets, packets[0].length), 0.001f);
        seq = new SequencePreprocessor(new Preprocessor[]{proB, proA});
        expected = new float[]{10f, 20f, 30f, 1f, 2f, 3f, 20f, 30f, 40f, 1f, 2f, 3f};
        Assert.assertArrayEquals(expected, seq.preprocessState(state, packets, packets[0].length), 0.001f);
    }
}

class PreprocessorA implements Preprocessor {

    @Override
    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        float[] result = new float[vehicleState.length * 2 + otherStates.length * statePacketLength];
        for (int i = 0; i < vehicleState.length * 2; i++) {
            result[i] = vehicleState[i % vehicleState.length];
        }
        for (int i = 0; i < otherStates.length; i++) {
            for (int j = 0; j < statePacketLength; j++) {
                result[vehicleState.length * 2 + i * statePacketLength + j] = otherStates[i][j];
            }
        }
        return result;
    }

    @Override
    public int getStateLength(int inputStateLength, int inputStatePacketLength) {
        return 2 * inputStateLength;
    }

    @Override
    public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
        return inputStatePacketLength;
    }
}

class PreprocessorB implements Preprocessor {

    @Override
    public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        float[] result = new float[otherStates.length * (vehicleState.length + statePacketLength)];
        for (int i = 0; i < otherStates.length; i++) {
            for (int j = 0; j < statePacketLength; j++) {
                result[i * (statePacketLength + vehicleState.length) + j] = otherStates[i][j];
            }
            for (int j = 0; j < vehicleState.length; j++) {
                result[statePacketLength + i * (statePacketLength + vehicleState.length) + j] = vehicleState[j];
            }
        }
        return result;
    }

    @Override
    public int getStateLength(int inputStateLength, int inputStatePacketLength) {
        return 0;
    }

    @Override
    public int getStatePacketLength(int inputStateLength, int inputStatePacketLength) {
        return inputStateLength + inputStatePacketLength;
    }
}


