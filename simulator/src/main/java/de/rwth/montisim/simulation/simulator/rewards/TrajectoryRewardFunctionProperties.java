package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("trajectory_reward")
public class TrajectoryRewardFunctionProperties extends RewardFunctionProperties {

  float reward = 100;
  float distance_max = 5;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new TrajectoryRewardFunction(vehicles, tickDuration, reward, distance_max);
  }
}
