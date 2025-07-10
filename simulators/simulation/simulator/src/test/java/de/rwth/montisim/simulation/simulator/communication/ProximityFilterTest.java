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
        state[21] = -1;
        state[22] = 2;

        // Create other states
        float[][] otherStates = new float[3][];
        for (int i = 0; i < 3; i++) {
            otherStates[i] = new float[25];
        }
        otherStates[0][21] = 9;
        otherStates[0][22] = 100;
        otherStates[1][21] = -1;
        otherStates[1][22] = 1;
        otherStates[2][21] = 2;
        otherStates[2][22] = 2;

        // Test filtering out all other vehicles:
        filter = new ProximityFilter(0, false);
        float[] result = filter.preprocessState(state, otherStates, 25);
        Assert.assertArrayEquals(state, result, 0.001f);
        result = filter.preprocessState(state, new float[0][], 25);
        Assert.assertArrayEquals(state, result, 0.001f);

        filter = new ProximityFilter(2, false);

        // Test filtering with no other vehicles
        float[] expected = new float[25 + 2 * 25];
        for (int i = 0; i < 25; i++) {
            expected[i] = state[i];
        }
        for (int i = 0; i < 2 * 25; i++) {
            expected[25 + i] = 0;
        }
        result = filter.preprocessState(state, new float[0][], 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with one vehicle
        for (int i = 0; i < 25; i++) {
            expected[25 + i] = otherStates[1][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1]}, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with two vehicles
        for (int i = 0; i < 25; i++) {
            expected[25 + 25 + i] = otherStates[2][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1], otherStates[2]}, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with three vehicles
        result = filter.preprocessState(state, otherStates, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test indicator with no other vehicles
        filter = new ProximityFilter(2, true);
        expected = new float[25 + 2 * (25 + 1)];
        for (int i = 0; i < 25; i++) {
            expected[i] = state[i];
        }
        for (int i = 0; i < 2 * (25 + 1); i++) {
            expected[25 + i] = 0;
        }
        result = filter.preprocessState(state, new float[0][], 25);
        Assert.assertArrayEquals(expected, result, 0.001f);
        // Test indicator with one vehicle
        for (int i = 0; i < (25+1); i++) {
            if (i == (25+1) - 1) expected[25 + i] = 1;
            else expected[25 + i] = otherStates[1][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1]}, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with two vehicles
        for (int i = 0; i < (25 + 1); i++) {
            if (i == (25+1) - 1) expected[25 + (25 + 1) + i] = 1;
            else expected[25 + (25 + 1) + i] = otherStates[2][i];
        }
        result = filter.preprocessState(state, new float[][]{otherStates[1], otherStates[2]}, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);

        // Test filtering with three vehicles
        result = filter.preprocessState(state, otherStates, 25);
        Assert.assertArrayEquals(expected, result, 0.001f);
    }
}