package de.rwth.montisim.simulation.simulator.communication;

import org.junit.Assert;
import org.junit.Test;

public class TrajectoryFilterTest {

  @Test
  public void testKeepOrder() {
    float[] vehicleState = new float[25];
    vehicleState[0] = 5;
    vehicleState[1] = 10;
    vehicleState[20] = 2;
    vehicleState[24] = 5;

    float[] otherState1 = new float[25];
    otherState1[0] = 5;
    otherState1[10] = 10;
    otherState1[1] = 10;
    otherState1[11] = 10;
    otherState1[20] = 2;
    otherState1[22] = 10;
    otherState1[24] = 5;

    float[] otherState2 = new float[25];
    otherState2[0] = 5;
    otherState2[10] = -10;
    otherState2[1] = 10;
    otherState2[11] = -10;
    otherState2[20] = 2;
    otherState2[22] = -10;
    otherState2[24] = 5;

    float[][] otherStates = new float[][] { otherState1, otherState2 };

    float[] result = new TrajectoryFilter(1.5f, 4.5f, 25, 2, false).preprocessState(vehicleState, otherStates, 25);

    float[] correct_solution = new float[25 * 3];
    System.arraycopy(vehicleState, 0, correct_solution, 0, 25);
    System.arraycopy(otherState1, 0, correct_solution, 25, 25);
    System.arraycopy(otherState2, 0, correct_solution, 2 * 25, 25);

    Assert.assertArrayEquals(correct_solution, result, 0);
  }

  @Test
  public void testOneIntersection() {
    float[] vehicleState = new float[25];
    vehicleState[0] = 5;
    vehicleState[1] = 10;
    vehicleState[20] = 2;
    vehicleState[24] = 5;

    float[] otherState1 = new float[25];
    otherState1[0] = 5;
    otherState1[10] = 10;
    otherState1[1] = 10;
    otherState1[11] = 10;
    otherState1[20] = 2;
    otherState1[22] = 10;
    otherState1[24] = 5;

    float[] otherState2 = new float[25];
    otherState2[0] = 5;
    otherState2[10] = -5;
    otherState2[1] = 10;
    otherState2[11] = 0;
    otherState2[20] = 2;
    otherState2[22] = -10;
    otherState2[24] = 5;

    float[][] otherStates = new float[][] { otherState1, otherState2 };

    float[] result = new TrajectoryFilter(1.5f, 4.5f, 1000, 2, false).preprocessState(vehicleState, otherStates, 25);

    float[] correct_solution = new float[25 * 3];
    System.arraycopy(vehicleState, 0, correct_solution, 0, 25);
    System.arraycopy(otherState2, 0, correct_solution, 25, 25);
    System.arraycopy(otherState1, 0, correct_solution, 2 * 25, 25);

    Assert.assertArrayEquals(correct_solution, result, 0);
  }

  @Test
  public void testFilterLessVehicles() {
    float[] vehicleState = new float[25];
    vehicleState[0] = 5;
    vehicleState[1] = 10;
    vehicleState[20] = 2;
    vehicleState[24] = 5;

    float[] otherState1 = new float[25];
    otherState1[0] = 5;
    otherState1[10] = 10;
    otherState1[1] = 10;
    otherState1[11] = 10;
    otherState1[20] = 2;
    otherState1[22] = 10;
    otherState1[24] = 5;

    float[][] otherStates = new float[][] { otherState1 };

    float[] result = new TrajectoryFilter(1.5f, 4.5f, 25, 2, true).preprocessState(vehicleState, otherStates, 25);

    float[] correct_solution = new float[25 + 2 * 26];
    System.arraycopy(vehicleState, 0, correct_solution, 0, 25);
    System.arraycopy(otherState1, 0, correct_solution, 25, 25);
    correct_solution[25 + 25] = 1;

    Assert.assertArrayEquals(correct_solution, result, 0);
  }

  @Test
  public void testKeepOrderMoreVehicles() {
    float[] vehicleState = new float[25];
    vehicleState[0] = 5;
    vehicleState[1] = 10;
    vehicleState[20] = 2;
    vehicleState[24] = 5;

    float[] otherState1 = new float[25];
    otherState1[0] = 5;
    otherState1[10] = 10;
    otherState1[1] = 10;
    otherState1[11] = 10;
    otherState1[20] = 2;
    otherState1[22] = 10;
    otherState1[24] = 5;

    float[] otherState2 = new float[25];
    otherState2[0] = 5;
    otherState2[10] = -10;
    otherState2[1] = 10;
    otherState2[11] = -10;
    otherState2[20] = 2;
    otherState2[22] = -10;
    otherState2[24] = 5;

    float[][] otherStates = new float[][] { otherState1, otherState2 };

    float[] result = new TrajectoryFilter(1.5f, 4.5f, 25, 1, true).preprocessState(vehicleState, otherStates, 25);

    float[] correct_solution = new float[25 + 26];
    System.arraycopy(vehicleState, 0, correct_solution, 0, 25);
    System.arraycopy(otherState1, 0, correct_solution, 25, 25);
    correct_solution[25 + 25] = 1;

    Assert.assertArrayEquals(correct_solution, result, 0);
  }

}
