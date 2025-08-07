package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("trajectory_reward")
public class TrajectoryRewardFunctionProperties extends RewardFunctionProperties {

  public float TRAJECTORY_FOLLOWING_REWARD = 1;
  public float TOTAL_PATH_PROGRESS_REWARD_SCALING = 10;
  public float PATH_PROGRESS_DERIVATIVE_REWARD_SCALING = 100;
  public float distance_max = 5;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new TrajectoryRewardFunction(vehicles, tickDuration, TRAJECTORY_FOLLOWING_REWARD, TOTAL_PATH_PROGRESS_REWARD_SCALING, PATH_PROGRESS_DERIVATIVE_REWARD_SCALING, distance_max);
  }
}
