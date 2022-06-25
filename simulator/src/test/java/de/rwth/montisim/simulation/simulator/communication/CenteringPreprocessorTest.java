package de.rwth.montisim.simulation.simulator.communication;

import org.junit.Assert;
import org.junit.Test;

public class CenteringPreprocessorTest {

  @Test
  public void testSingleVehicle() {
    float[] vehicleState = new float[25];
    // set Position
    vehicleState[21] = 5;
    vehicleState[22] = 6;
    // set a few Trajectory Points
    vehicleState[0] = 5;
    vehicleState[10] = 6;
    vehicleState[1] = 5;
    vehicleState[11] = 7;
    vehicleState[2] = 6;
    vehicleState[12] = 7;
    vehicleState[3] = 6;
    vehicleState[13] = 6;
    vehicleState[4] = 6;
    vehicleState[14] = 5;
    vehicleState[5] = 5;
    vehicleState[15] = 5;
    vehicleState[6] = 4;
    vehicleState[16] = 5;
    // set Trajectory Length
    vehicleState[20] = 7;

    float[] solution = new float[25];
    // set Position
    solution[21] = 0;
    solution[22] = 0;
    // set a few Trajectory Points
    solution[0] = 0;
    solution[10] = 0;
    solution[1] = 0;
    solution[11] = 1;
    solution[2] = 1;
    solution[12] = 1;
    solution[3] = 1;
    solution[13] = 0;
    solution[4] = 1;
    solution[14] = -1;
    solution[5] = 0;
    solution[15] = -1;
    solution[6] = -1;
    solution[16] = -1;
    // set Trajectory Length
    solution[20] = 7;

    float[] result = new CenteringPreprocessor().preprocessState(vehicleState, new float[0][], 25);

    Assert.assertArrayEquals(solution, result, 0);
  }

  @Test
  public void testTwoVehicles() {
    float[] vehicleState = new float[25];
    // set Position
    vehicleState[21] = 5;
    vehicleState[22] = 6;

    float[][] otherStates = new float[1][];
    float[] otherState = new float[25];
    // set Position
    otherState[21] = 25;
    otherState[22] = -25;
    // set a few Trajectory Points
    otherState[0] = 5;
    otherState[10] = 6;
    otherState[1] = 5;
    otherState[11] = 7;
    otherState[2] = 6;
    otherState[12] = 7;
    otherState[3] = 6;
    otherState[13] = 6;
    otherState[4] = 6;
    otherState[14] = 5;
    otherState[5] = 5;
    otherState[15] = 5;
    otherState[6] = 4;
    otherState[16] = 5;
    // set Trajectory Length
    otherState[20] = 7;
    otherStates[0] = otherState;

    float[] solution = new float[25 + 25];
    // set Positions
    solution[21] = 0;
    solution[22] = 0;
    solution[25 + 21] = 20;
    solution[25 + 22] = -31;
    // set a few Trajectory Points
    solution[25 + 0] = 0;
    solution[25 + 10] = 0;
    solution[25 + 1] = 0;
    solution[25 + 11] = 1;
    solution[25 + 2] = 1;
    solution[25 + 12] = 1;
    solution[25 + 3] = 1;
    solution[25 + 13] = 0;
    solution[25 + 4] = 1;
    solution[25 + 14] = -1;
    solution[25 + 5] = 0;
    solution[25 + 15] = -1;
    solution[25 + 6] = -1;
    solution[25 + 16] = -1;
    // set Trajectory Length
    solution[25 + 20] = 7;

    float[] result = new CenteringPreprocessor().preprocessState(vehicleState, otherStates, 25);

    Assert.assertArrayEquals(solution, result, 0);
  }

}
