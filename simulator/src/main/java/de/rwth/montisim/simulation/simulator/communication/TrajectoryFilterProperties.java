package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

@Typed("trajectory")
public class TrajectoryFilterProperties extends PreprocessorProperties{


  public float VEHICLE_WIDTH;
  public float VEHICLE_LENGTH;
  public int MAX_STATES_PER_VEHICLE = 25;

  @Override
  public Preprocessor build() {
    return new TrajectoryFilter(VEHICLE_WIDTH, VEHICLE_LENGTH, MAX_STATES_PER_VEHICLE);
  }
}
