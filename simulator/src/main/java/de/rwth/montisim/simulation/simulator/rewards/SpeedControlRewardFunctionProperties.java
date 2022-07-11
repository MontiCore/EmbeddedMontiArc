package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

public class SpeedControlRewardFunctionProperties extends RewardFunctionProperties {

  public float VELOCITY_DIFFERENCE_SCALING = 1;
  public float VELOCITY_SUB_MAXIMUM_REWARD = 0.5f;
  public float STANDING_PUNISHMENT = -10;
  public float CONTROL_EFFORT_SCALING = 1;
  public float CONTROL_EFFORT_DERIVATIVE_SCALING = 0.05f;
  public float velocity_max = 40;
  public float velocity_desired = 20;
  public float standing_threshold = 0.5f;
  public int standing_punishment_from_step = 5;
  public float standing_punishment_to_min_path_distance = 2.5f;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new SpeedControlRewardFunction(vehicles, tickDuration, VELOCITY_DIFFERENCE_SCALING, VELOCITY_SUB_MAXIMUM_REWARD, STANDING_PUNISHMENT, CONTROL_EFFORT_SCALING, CONTROL_EFFORT_DERIVATIVE_SCALING, velocity_max, velocity_desired, standing_threshold, standing_punishment_from_step, standing_punishment_to_min_path_distance);
  }
}
