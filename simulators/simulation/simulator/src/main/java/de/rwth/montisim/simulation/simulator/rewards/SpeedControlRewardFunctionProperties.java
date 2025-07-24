package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("speed_control_reward")
public class SpeedControlRewardFunctionProperties extends VariableSpeedControlRewardFunctionProperties {
  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new SpeedControlRewardFunction(vehicles, tickDuration, VELOCITY_DIFFERENCE_REWARD_SCALING, VELOCITY_SUB_MAXIMUM_REWARD, STANDING_REWARD, LINEAR_ACCELERATION_REWARD_SCALING, LINEAR_JERK_REWARD_SCALING, ANGULAR_ACCELERATION_REWARD_SCALING, ANGULAR_JERK_REWARD_SCALING, velocity_max, velocity_desired, standing_threshold, standing_punishment_from_step, standing_punishment_to_min_path_distance);
  }
}
