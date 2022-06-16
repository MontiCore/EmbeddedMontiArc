package de.rwth.montisim.simulation.simulator.communication;

import junit.framework.TestCase;
import org.junit.Assert;

/**
 * Unit test, that tests the behavior of the ProximityFilter.
 * @see de.rwth.montisim.simulation.simulator.communication.ProximityFilter;
 */
public class ProximityFilterTest extends TestCase {

    public void testPreprocessState() {
        ProximityFilter filter = new ProximityFilter(0, false);
        // Create state
        float[] state = new float[25];
        for (int i = 0; i < 21; i++) state[i] = 0;
        state[22] = -1;
        state[23] = 2;

        // Create other states
        float[][] otherStates = new float[3][];
        for (int i = 0; i < 3; i++) {
            otherStates[i] = new float[4];
            for (int j = 0; j < 2; j++) {
                otherStates[i][2 + j] = 0;
            }
        }
        otherStates[0][0] = 9;
        otherStates[0][1] = 100;
        otherStates[1][0] = -1;
        otherStates[1][1] = 1;
        otherStates[2][0] = 2;
        otherStates[2][1] = 2;

        // Test filtering out all other vehicles:
        float[] result = filter.preprocessState(state, otherStates, 4);
        Assert.assertArrayEquals(state, result, 0.001f);
        result = filter.preprocessState(state, new float[0][], 4);
        Assert.assertArrayEquals(state, result, 0.001f);

        filter = new ProximityFilter(2, false);

        // Test filtering with no other vehicles
        float[] expected = new float[25 + 2 * 4];
        for (int i = 0; i < 25; i++) {
            expected[i] = state[i];
        }
        for (int i = 0; i < 2 * 4; i++) {
            expected[25 + i] = 0;
        }
        result = filter.preprocessState(state, new float[0][], 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with one vehicle
        for (int i = 0; i < 4; i++) {
            expected[25 + i] = otherStates[1][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1]}, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with two vehicles
        for (int i = 0; i < 4; i++) {
            expected[25 + 4 + i] = otherStates[2][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1], otherStates[2]}, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with three vehicles
        result = filter.preprocessState(state, otherStates, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test indicator with no other vehicles
        filter = new ProximityFilter(2, true);
        expected = new float[25 + 2 * 5];
        for (int i = 0; i < 25; i++) {
            expected[i] = state[i];
        }
        for (int i = 0; i < 2 * 5; i++) {
            expected[25 + i] = 0;
        }
        result = filter.preprocessState(state, new float[0][], 4);
        Assert.assertArrayEquals(expected, result, 0.001f);
        // Test indicator with one vehicle
        for (int i = 0; i < 5; i++) {
            if (i == 5 - 1) expected[25 + i] = 1;
            else expected[25 + i] = otherStates[1][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1]}, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with two vehicles
        for (int i = 0; i < 5; i++) {
            if (i == 5 -1 ) expected[25 + 5 + i] = 1;
            else expected[25 + 5 + i] = otherStates[2][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1], otherStates[2]}, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with three vehicles
        result = filter.preprocessState(state, otherStates, 4);
        Assert.assertArrayEquals(expected, result, 0.001f);
    }
}