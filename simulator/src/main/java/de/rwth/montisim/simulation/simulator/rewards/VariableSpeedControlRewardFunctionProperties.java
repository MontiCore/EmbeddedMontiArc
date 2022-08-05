package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("variable_speed_control_reward")
public class VariableSpeedControlRewardFunctionProperties extends RewardFunctionProperties {

  public float VELOCITY_DIFFERENCE_REWARD_SCALING = 1;
  public float VELOCITY_SUB_MAXIMUM_REWARD = 0.5f;
  public float STANDING_REWARD = -10;
  public float LINEAR_ACCELERATION_REWARD_SCALING = 1;
  public float LINEAR_JERK_REWARD_SCALING = 0.05f;
  public float ANGULAR_ACCELERATION_REWARD_SCALING = 0;
  public float ANGULAR_JERK_REWARD_SCALING = 0;
  public float velocity_max = 40;
  public float velocity_desired = 20;
  public float standing_threshold = 0.5f;
  public int standing_punishment_from_step = 5;
  public float standing_punishment_to_min_path_distance = 2.5f;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new VariableSpeedControlRewardFunction(vehicles, tickDuration, VELOCITY_DIFFERENCE_REWARD_SCALING, VELOCITY_SUB_MAXIMUM_REWARD, STANDING_REWARD, LINEAR_ACCELERATION_REWARD_SCALING, LINEAR_JERK_REWARD_SCALING, ANGULAR_ACCELERATION_REWARD_SCALING, ANGULAR_JERK_REWARD_SCALING, velocity_max, velocity_desired, standing_threshold, standing_punishment_from_step, standing_punishment_to_min_path_distance);
  }

}
