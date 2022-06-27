package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

@Typed("trajectory_reward")
public class TrajectoryRewardFunctionProperties extends RewardFunctionProperties {

  float reward = 100;
  float distance_max = 5;

  @Override
  public RewardFunction build(Navigation[] navigations, Vehicle[] vehicles) {
    return new TrajectoryRewardFunction(navigations, vehicles, reward, distance_max);
  }
}
