package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

import java.util.Optional;

@Typed("platooning_reward")
public class PlatooningRewardFunctionProperties extends RewardFunctionProperties {

  public float reward = 1;
  public float gap_max = 20;
  public float gap_desired = 10;
  public float velocity_max = 40;
  public float velocity_desired = 20;

  public Optional<StaticCollisionsRewardFunctionProperties> static_collision_reward_properties;
  public Optional<VehicleCollisionsRewardFunctionProperties> vehicle_collision_reward_properties;
  public Optional<TrajectoryRewardFunctionProperties> trajectory_reward_properties;

  @Override
  public RewardFunction build(Navigation[] navigations, Vehicle[] vehicles) {
    RewardFunction[] rewardFunctionsArray = new RewardFunction[4];
    rewardFunctionsArray[0] = new PlatooningRewardFunction(navigations, vehicles, reward, gap_max, gap_desired, velocity_max, velocity_desired);

    if (static_collision_reward_properties.isPresent()) {
      rewardFunctionsArray[0] = static_collision_reward_properties.get().build(navigations, vehicles);
    }
    else {
      // default scrp
      StaticCollisionsRewardFunctionProperties scrp = new StaticCollisionsRewardFunctionProperties();
      scrp.reward = -500;
      rewardFunctionsArray[1] = scrp.build(navigations, vehicles);
    }

    if (vehicle_collision_reward_properties.isPresent()) {
      rewardFunctionsArray[0] = vehicle_collision_reward_properties.get().build(navigations, vehicles);
    }
    else {
      // default scrp
      VehicleCollisionsRewardFunctionProperties vcrp = new VehicleCollisionsRewardFunctionProperties();
      vcrp.reward = -500;
      rewardFunctionsArray[2] = vcrp.build(navigations, vehicles);
    }

    if (trajectory_reward_properties.isPresent()) {
      rewardFunctionsArray[0] = trajectory_reward_properties.get().build(navigations, vehicles);
    }
    else {
      // default scrp
      TrajectoryRewardFunctionProperties trp = new TrajectoryRewardFunctionProperties();
      trp.reward = 1;
      trp.distance_max = 5;
      rewardFunctionsArray[3] = trp.build(navigations, vehicles);
    }

    return new SequenceRewardFunction(navigations, vehicles, rewardFunctionsArray);
  }
}
