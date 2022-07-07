package de.rwth.montisim.simulation.simulator.communication;

import junit.framework.TestCase;
import org.junit.Assert;

public class AnglePreprocessorTest extends TestCase {

    public void testPreprocessState() {
        // Create State
        float[] vehicleState = new float[]{1, 1, 0, 7};
        float[][] statePackets = new float[][]{{0, -3, -45}, {4, 7, 180}, {1, 4, 270}};
        // Test
        AnglePreprocessor preprocessor = new AnglePreprocessor(2, 2);
        float[] expected = new float[]{1, 1, 0, 1, 7, 0, -3, -0.7071067f, 0.7071067f, 4, 7, 0, -1, 1, 4, -1, 0};
        float[] actual = preprocessor.preprocessState(vehicleState, statePackets, 3);
        Assert.assertArrayEquals(expected, actual, 0.001f);
        Assert.assertEquals(5, preprocessor.getStateLength(vehicleState.length, statePackets[0].length));
        Assert.assertEquals(4, preprocessor.getStatePacketLength(vehicleState.length, statePackets[0].length));
    }

}