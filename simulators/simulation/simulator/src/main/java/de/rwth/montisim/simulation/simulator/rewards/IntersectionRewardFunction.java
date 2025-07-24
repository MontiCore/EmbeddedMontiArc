package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward Function that evaluates a vehicles' performance in intersection situations.
 * This reward function is inspired by the paper:
 * - A Control Method with Reinforcement Learning for Urban
 * Un-Signalized Intersection in Hybrid Traffic Environment by Shi et al.
 */
public class IntersectionRewardFunction extends RewardFunction {
  private final float VEHICLE_COLLISION_FACTOR;
  private final float STATIC_COLLISION_FACTOR;
  private final float TRAJECTORY_REWARD_FACTOR;
  private final float ANGULAR_JERK_REWARD_FACTOR;
  private final float ANGULAR_ACC_REWARD_FACTOR;
  private final float LINEAR_JERK_REWARD_FACTOR;
  private final float LINEAR_ACC_REWARD_FACTOR;
  private final float TRAJECTORY_FOLLOWING_REWARD;
  private final float TOTAL_PATH_PROGRESS_REWARD_SCALING;
  private final float PATH_PROGRESS_DERIVATIVE_REWARD_SCALING;
  private final float DISTANCE_MAX;
  private final float VELOCITY_DIFFERENCE_SCALING;
  private final float VELOCITY_SUB_MAXIMUM_REWARD;
  private final float STANDING_PUNISHMENT;
  private final float velocity_max;
  private final float velocity_desired;
  private final float standing_threshold;
  private final int standing_punishment_from_step;
  private final float standing_punishment_to_min_path_distance;

  private final SumRewardFunction reward;

  public IntersectionRewardFunction(Vehicle[] vehicles, Duration tickDuration, float vehicle_collision_factor, float static_collision_factor, float trajectory_reward_factor, float angular_jerk_reward_factor, float angular_acc_reward_factor, float linear_jerk_reward_factor, float linear_acc_reward_factor, float trajectory_following_reward, float total_path_progress_reward_scaling, float path_progress_derivative_reward_scaling, float distance_max, float velocity_difference_scaling, float velocity_sub_maximum_reward, float standing_punishment, float velocity_max, float velocity_desired, float standing_threshold, int standing_punishment_from_step, float standing_punishment_to_min_path_distance) {
    super(vehicles, tickDuration);
    this.VEHICLE_COLLISION_FACTOR = vehicle_collision_factor;
    this.STATIC_COLLISION_FACTOR = static_collision_factor;
    this.TRAJECTORY_REWARD_FACTOR = trajectory_reward_factor;
    this.ANGULAR_JERK_REWARD_FACTOR = angular_jerk_reward_factor;
    this.ANGULAR_ACC_REWARD_FACTOR = angular_acc_reward_factor;
    this.LINEAR_JERK_REWARD_FACTOR = linear_jerk_reward_factor;
    this.LINEAR_ACC_REWARD_FACTOR = linear_acc_reward_factor;
    this.TRAJECTORY_FOLLOWING_REWARD = trajectory_following_reward;
    this.TOTAL_PATH_PROGRESS_REWARD_SCALING = total_path_progress_reward_scaling;
    this.PATH_PROGRESS_DERIVATIVE_REWARD_SCALING = path_progress_derivative_reward_scaling;
    this.DISTANCE_MAX = distance_max;
    this.VELOCITY_DIFFERENCE_SCALING = velocity_difference_scaling;
    this.VELOCITY_SUB_MAXIMUM_REWARD = velocity_sub_maximum_reward;
    this.STANDING_PUNISHMENT = standing_punishment;
    this.velocity_max = velocity_max;
    this.velocity_desired = velocity_desired;
    this.standing_threshold = standing_threshold;
    this.standing_punishment_from_step = standing_punishment_from_step;
    this.standing_punishment_to_min_path_distance = standing_punishment_to_min_path_distance;

    // Build Reward Function

    VehicleCollisionsRewardFunction vehColReward = new VehicleCollisionsRewardFunction(vehicles, tickDuration, 1);
    StaticCollisionsRewardFunction statColReward = new StaticCollisionsRewardFunction(vehicles, tickDuration, 1);
    TrajectoryRewardFunction trajReward = new TrajectoryRewardFunction(vehicles, tickDuration, TRAJECTORY_FOLLOWING_REWARD, TOTAL_PATH_PROGRESS_REWARD_SCALING, PATH_PROGRESS_DERIVATIVE_REWARD_SCALING, DISTANCE_MAX);
    SpeedControlRewardFunction speedReward = new SpeedControlRewardFunction(vehicles, tickDuration, VELOCITY_DIFFERENCE_SCALING, VELOCITY_SUB_MAXIMUM_REWARD, STANDING_PUNISHMENT, LINEAR_ACC_REWARD_FACTOR, LINEAR_JERK_REWARD_FACTOR, ANGULAR_ACC_REWARD_FACTOR, ANGULAR_JERK_REWARD_FACTOR, this.velocity_max, this.velocity_desired, this.standing_threshold, this.standing_punishment_from_step, this.standing_punishment_to_min_path_distance);
    RewardFunction[] components = new RewardFunction[] { vehColReward, statColReward, trajReward, speedReward };
    float[] factors = new float[] { VEHICLE_COLLISION_FACTOR, STATIC_COLLISION_FACTOR, TRAJECTORY_REWARD_FACTOR, 1f };
    reward = new SumRewardFunction(vehicles, tickDuration, components, factors);

  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    return reward.getRewardForVehicle(vehicle_index, step);
  }
}
