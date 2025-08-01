package de.rwth.montisim.simulation.simulator.communication;

/**
 * Preprocessing strategy that centers all coordinates in the states of each vehicle around the current location of a given vehicle.
 * This preprocessor returns a state array of constant size.
 * The size of the state is: vehicleState + statePacketLength * otherStates
 */
public class CenteringPreprocessor implements Preprocessor {
  final static String STATE_PACKET_LENGTH_ERROR = "Preprocessor received different state packet lengths from different vehicles.";

  @Override
  public float[] preprocessState(float[] vehicleState, float[][] otherStates, int statePacketLength) {
        /*
            This operates on the current assumption that the vehicleState and StatePacketLengths
            have the same format as in Commit 4b1dea4fdf84ca2501585b154274d053de2caee8.
            These are just some simple checks that detect at least some change in format.
            If the format changes, this method body has to be adjusted!
         */
    assert vehicleState.length == 25;
    assert statePacketLength == 25;
    for (int i = 0; i < otherStates.length; i++)
      assert otherStates[i].length == statePacketLength;

    // Get the position of the current vehicle
    float real_center_x = vehicleState[21], real_center_y = vehicleState[22];

    // Combine the states
    float[] result = new float[vehicleState.length + otherStates.length * statePacketLength];

    // Copy vehicleState into result while centering
    int traj_length = (int) vehicleState[20];
    for (int i = 0; i < traj_length; i++) {
      result[i] = vehicleState[i] - real_center_x;
    }
    for (int i = 10; i < 10 + traj_length; i++) {
      result[i] = vehicleState[i] - real_center_y;
    }
    result[20] = traj_length;
    result[21] = vehicleState[21] - real_center_x;
    result[22] = vehicleState[22] - real_center_y;
    for (int i = 23; i < vehicleState.length; i++) {
      result[i] = vehicleState[i];
    }

    // Copy all others vehicles states and center them
    for (int j = 0; j < otherStates.length; j++) {
      traj_length = (int) otherStates[j][20];
      for (int i = 0; i < traj_length; i++) {
        result[vehicleState.length + j * statePacketLength + i] = otherStates[j][i] - real_center_x;
      }
      for (int i = 10; i < 10 + traj_length; i++) {
        result[vehicleState.length + j * statePacketLength + i] = otherStates[j][i] - real_center_y;
      }
      result[vehicleState.length + j * statePacketLength + 20] = traj_length;
      result[vehicleState.length + j * statePacketLength + 21] = otherStates[j][21] - real_center_x;
      result[vehicleState.length + j * statePacketLength + 22] = otherStates[j][22] - real_center_y;
      for (int i = 23; i < vehicleState.length; i++) {
        result[vehicleState.length + j * statePacketLength + i] = otherStates[j][i];
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
    return inputStateLength;
  }
}
