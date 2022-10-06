package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.List;
import java.util.Optional;
import java.util.Stack;

/**
 * Reward Function that evaluates the behaviour of vehicles in a platoon.
 * This reward function is inspired by the following papers:
 * - Reinforcement Learning Based Approach for
 * Multi-Vehicle Platooning Problem with Nonlinear
 * Dynamic Behavior, Farag et. al.
 * - Platoon control of connected autonomous
 * vehicles: A distributed reinforcement
 * learning method by consensus, Liu et. al.
 */
public class PlatooningRewardFunction extends RewardFunction {

  private final float GAP_DISTANCE_REWARD_SCALING;
  private final float GAP_SUB_MAXIMUM_REWARD;
  private final VariableSpeedControlRewardFunction speed_control_reward_function;
  private final float gap_max;
  private final float gap_desired;

  private final float velocity_desired;
  private final int look_ahead;

  /**
   * Initializes a new Platoon Reward Function
   *
   * @param vehicles                      Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration                  Duration between two Updates of the Simulator.
   * @param speed_control_reward_function The Speed Control Component variable depending on if a vehicle is a leader of a follower in the platoon.
   * @param GAP_DISTANCE_REWARD_SCALING   Scaling of the reward given to the distance to the next vehicle.
   * @param GAP_SUB_MAXIMUM_REWARD        Reward given, if the vehicle is less than the maximum distance behind the preceding vehicle.
   * @param gap_max                       Maximum allowed gap between vehicles.
   * @param gap_desired                   Desired gap between vehicles.
   */
  public PlatooningRewardFunction(Vehicle[] vehicles, Duration tickDuration, VariableSpeedControlRewardFunction speed_control_reward_function, float GAP_DISTANCE_REWARD_SCALING, float GAP_SUB_MAXIMUM_REWARD, float gap_max, float gap_desired, int look_ahead) {
    super(vehicles, tickDuration);
    this.GAP_DISTANCE_REWARD_SCALING = GAP_DISTANCE_REWARD_SCALING;
    this.GAP_SUB_MAXIMUM_REWARD = GAP_SUB_MAXIMUM_REWARD;
    this.speed_control_reward_function = speed_control_reward_function;
    this.velocity_desired = speed_control_reward_function.velocity_desired;
    this.gap_max = gap_max;
    this.gap_desired = gap_desired;
    this.look_ahead = look_ahead;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;

    Optional<Integer> preceding_index_optional = getPrecedingVehicleIndex(vehicle_index);
    if (preceding_index_optional.isPresent()) { // current vehicle is following another vehicle
      int preceding_index = preceding_index_optional.get();

      // punish distance
      Vec2 vehicle_position = (Vec2) this.positions[vehicle_index].get();
      Vec2 preceding_position = (Vec2) this.positions[preceding_index].get();
      double gap = vehicle_position.distance(preceding_position);
      reward -= GAP_DISTANCE_REWARD_SCALING * Math.pow((1 / this.gap_max) * (this.gap_desired - gap), 2);

      // reward below max gap
      if (this.gap_max >= gap)
        reward += GAP_SUB_MAXIMUM_REWARD;

      // set desired velocity to preceding vehicles speed
      speed_control_reward_function.velocity_desired = ((Double) this.velocities[preceding_index].get()).floatValue();
    }
    else { // current vehicle is not following another vehicle, thus is a leader
      // reset desired velocity
      speed_control_reward_function.velocity_desired = this.velocity_desired;
    }
    // reward speed control
    reward += speed_control_reward_function.getRewardForVehicle(vehicle_index, step);

    return reward;
  }

  /**
   * Determines the index of the vehicle preceding a given vehicle.
   *
   * @param vehicle_index The index of the vehicle that follows the preceding vehicle in the platoon.
   * @return The index of the preceding vehicle, if present.
   */
  private Optional<Integer> getPrecedingVehicleIndex(int vehicle_index) {
    // Intersections which the vehicle drives through
    Stack<Vec2> vehicle_targets_stack = this.navigations[vehicle_index].getTargets();
    List<Vec2> vehicle_targets = vehicle_targets_stack.subList(0, Math.min(vehicle_targets_stack.size(), look_ahead));

    float[] distances = new float[vehicles.length];
    boolean[] passes_through_node = new boolean[vehicles.length];
    boolean found_a_node = false;
    for (int vehicle_target_index = 0; vehicle_target_index < vehicle_targets.size() && !found_a_node; vehicle_target_index++) {
      Vec2 vehicle_target = vehicle_targets.get(vehicle_target_index);
      distances[vehicle_index] += vehicle_target.distance((Vec2) this.positions[vehicle_index].get());

      passes_through_node = new boolean[vehicles.length];

      for (int other_vehicle_index = 0; other_vehicle_index < vehicles.length; other_vehicle_index++) {
        if (other_vehicle_index == vehicle_index)
          continue;

        // Intersections which the other vehicle drives through
        Stack<Vec2> other_vehicle_targets_stack = this.navigations[other_vehicle_index].getTargets();
        List<Vec2> other_vehicle_targets = other_vehicle_targets_stack.subList(0, Math.min(other_vehicle_targets_stack.size(), look_ahead));

        for (int other_vehicle_target_index = 0; other_vehicle_target_index < other_vehicle_targets.size(); other_vehicle_target_index++) {
          Vec2 other_vehicle_target = other_vehicle_targets.get(other_vehicle_target_index);
          distances[other_vehicle_index] += other_vehicle_target.distance((Vec2) this.positions[other_vehicle_index].get());

          if (vehicle_target.equals(other_vehicle_target) && distances[vehicle_index] > distances[other_vehicle_index]) {
            found_a_node = true;
            passes_through_node[other_vehicle_index] = true;
          }
        }
      }
    }

    if(!found_a_node)
      return Optional.empty();

    int best_index = -1;
    for (int other_vehicle_index = 0; other_vehicle_index < vehicles.length; other_vehicle_index++) {
      if (other_vehicle_index == vehicle_index)
        continue;

      if (passes_through_node[other_vehicle_index]) {
        if (best_index == -1 || distances[other_vehicle_index] > distances[best_index])
          best_index = other_vehicle_index;
      }
    }
    return Optional.of(best_index);
  }
}
