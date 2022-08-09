package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("intersection_reward")
public class IntersectionRewardFunctionProperties extends RewardFunctionProperties {

    public float vehicle_collision_factor = -800;
    public float static_collision_factor = -600;
    public float trajectory_reward_factor = 1;
    public float angular_jerk_reward_factor= 0.5f;
    public float angular_acc_reward_factor = 10;
    public float linear_jerk_reward_factor = 0.05f;
    public float linear_acc_reward_factor = 1;
    public float trajectory_following_reward = 1;
    public float total_path_progress_reward_scaling = 10;
    public float path_progress_derivative_reward_scaling = 100;
    public float distance_max = 5;
    public float velocity_difference_scaling = 1;
    public float velocity_sub_maximum_reward = 0.05f;
    public float standing_punishment = -10;
    public float velocity_max = 40;
    public float velocity_desired = 20;
    public float standing_threshold = 0.5f;
    public int standing_punishment_from_step = 5;
    public float standing_punishment_to_min_path_distance = 2.5f;

    @Override
    public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
        return new IntersectionRewardFunction(vehicles, tickDuration, vehicle_collision_factor, static_collision_factor,
                trajectory_reward_factor, angular_jerk_reward_factor, angular_acc_reward_factor, linear_jerk_reward_factor,
                linear_acc_reward_factor, trajectory_following_reward, total_path_progress_reward_scaling,
                path_progress_derivative_reward_scaling, distance_max, velocity_difference_scaling,
                velocity_sub_maximum_reward, standing_punishment, velocity_max, velocity_desired, standing_threshold,
                standing_punishment_from_step, standing_punishment_to_min_path_distance);
    }
}
