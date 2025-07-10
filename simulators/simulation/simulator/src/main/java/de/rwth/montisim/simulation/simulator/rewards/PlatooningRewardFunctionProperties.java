package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Optional;

@Typed("platooning_reward")
public class PlatooningRewardFunctionProperties extends RewardFunctionProperties {

  public float GAP_DISTANCE_REWARD_SCALING = 20;
  public float GAP_SUB_MAXIMUM_REWARD = 1;
  public float gap_max = 20;
  public float gap_desired = 10;
  public int look_ahead = 2;

  public Optional<SpeedControlRewardFunctionProperties> speed_control_reward_properties = Optional.empty();
  public Optional<StaticCollisionsRewardFunctionProperties> static_collision_reward_properties = Optional.empty();
  public Optional<VehicleCollisionsRewardFunctionProperties> vehicle_collision_reward_properties = Optional.empty();
  public Optional<TrajectoryRewardFunctionProperties> trajectory_reward_properties = Optional.empty();

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    RewardFunction[] rewardFunctionsArray = new RewardFunction[4];

    VariableSpeedControlRewardFunction vscrf;
    if (speed_control_reward_properties.isPresent()) {
      vscrf = (VariableSpeedControlRewardFunction) ((VariableSpeedControlRewardFunctionProperties) speed_control_reward_properties.get()).build(vehicles, tickDuration);
    }
    else {
      // default vscrf
      vscrf = (VariableSpeedControlRewardFunction) (new VariableSpeedControlRewardFunctionProperties()).build(vehicles, tickDuration);
    }

    rewardFunctionsArray[0] = new PlatooningRewardFunction(vehicles, tickDuration, vscrf, GAP_DISTANCE_REWARD_SCALING, GAP_SUB_MAXIMUM_REWARD, gap_max, gap_desired, look_ahead);

    if (static_collision_reward_properties.isPresent()) {
      rewardFunctionsArray[1] = static_collision_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      StaticCollisionsRewardFunctionProperties scrp = new StaticCollisionsRewardFunctionProperties();
      scrp.STATIC_COLLISION_REWARD = -500;
      rewardFunctionsArray[1] = scrp.build(vehicles, tickDuration);
    }

    if (vehicle_collision_reward_properties.isPresent()) {
      rewardFunctionsArray[2] = vehicle_collision_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      VehicleCollisionsRewardFunctionProperties vcrp = new VehicleCollisionsRewardFunctionProperties();
      vcrp.VEHICLE_COLLISIONS_REWARD = -500;
      rewardFunctionsArray[2] = vcrp.build(vehicles, tickDuration);
    }

    if (trajectory_reward_properties.isPresent()) {
      rewardFunctionsArray[3] = trajectory_reward_properties.get().build(vehicles, tickDuration);
    }
    else {
      // default scrp
      TrajectoryRewardFunctionProperties trp = new TrajectoryRewardFunctionProperties();
      trp.distance_max = 5;
      rewardFunctionsArray[3] = trp.build(vehicles, tickDuration);
    }

    return new SumRewardFunction(vehicles, tickDuration, rewardFunctionsArray);
  }
}
