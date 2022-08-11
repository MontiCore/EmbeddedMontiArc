package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Optional;

@Typed("basic_reward")
public class BasicRewardFunctionProperties extends RewardFunctionProperties{

  public Optional<StaticCollisionsRewardFunctionProperties> static_collision_reward_properties = Optional.empty();
  public Optional<TrajectoryRewardFunctionProperties> trajectory_reward_properties = Optional.empty();
  public Optional<SpeedControlRewardFunctionProperties> speed_control_reward_properties = Optional.empty();

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    RewardFunction[] rewardFunctionsArray = new RewardFunction[3];

    if (static_collision_reward_properties.isPresent()) {
      rewardFunctionsArray[0] = static_collision_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      StaticCollisionsRewardFunctionProperties scrp = new StaticCollisionsRewardFunctionProperties();
      scrp.STATIC_COLLISION_REWARD = -500;
      rewardFunctionsArray[0] = scrp.build(vehicles, tickDuration);
    }

    if (trajectory_reward_properties.isPresent()) {
      rewardFunctionsArray[1] = trajectory_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      TrajectoryRewardFunctionProperties trp = new TrajectoryRewardFunctionProperties();
      trp.distance_max = 5;
      rewardFunctionsArray[1] = trp.build(vehicles, tickDuration);
    }

    if (speed_control_reward_properties.isPresent()) {
      rewardFunctionsArray[2] = speed_control_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      SpeedControlRewardFunctionProperties scrp = new SpeedControlRewardFunctionProperties();
      rewardFunctionsArray[2] = scrp.build(vehicles, tickDuration);
    }

    return new SumRewardFunction(vehicles, tickDuration, rewardFunctionsArray);
  }
}
