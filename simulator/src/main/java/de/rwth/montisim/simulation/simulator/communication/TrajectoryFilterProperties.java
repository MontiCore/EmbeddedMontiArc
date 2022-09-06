package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

@Typed("trajectory")
public class TrajectoryFilterProperties extends PreprocessorProperties{

  // from https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/user-docs/json-scenario-reference/Body-and-Wheel-Properties
  public float VEHICLE_WIDTH = 1.87f;
  public float VEHICLE_LENGTH = 4.871f;
  public int MAX_STATES_PER_VEHICLE = 25;
  public int maxNumberOfVehicles = 5;
  public boolean addIndicator = true;

  @Override
  public Preprocessor build() {
    return new TrajectoryFilter(VEHICLE_WIDTH, VEHICLE_LENGTH, MAX_STATES_PER_VEHICLE, maxNumberOfVehicles, addIndicator);
  }
}
