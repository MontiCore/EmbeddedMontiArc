package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.Pair;
import de.rwth.montisim.commons.utils.Vec2;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
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

  private final float PLATOONING_REWARD;
  private final float gap_max;
  private final float gap_desired;
  private final float velocity_max;
  private final float velocity_desired;

  /**
   * Initializes a new Platoon Reward Function
   *
   * @param vehicles          Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration      Duration between two Updates of the Simulator.
   * @param platooning_reward The scaled reward.
   * @param gap_max           Maximum allowed gap between vehicles.
   * @param gap_desired       Desired gap between vehicles.
   * @param velocity_max      Maximum allowed velocity.
   * @param velocity_desired  Desired velocity.
   */
  public PlatooningRewardFunction(Vehicle[] vehicles, Duration tickDuration, float platooning_reward, float gap_max, float gap_desired, float velocity_max, float velocity_desired) {
    super(vehicles, tickDuration);
    this.PLATOONING_REWARD = platooning_reward;
    this.gap_max = gap_max;
    this.gap_desired = gap_desired;
    this.velocity_max = velocity_max;
    this.velocity_desired = velocity_desired;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;

    Optional<Integer> preceding_index_optional = getPrecedingVehicleIndex(vehicle_index);
    if (preceding_index_optional.isPresent()) {
      int preceding_index = preceding_index_optional.get();

      // punish distance
      Vec2 vehicle_position = this.positions[vehicle_index];
      Vec2 preceding_position = this.positions[preceding_index];
      double gap = vehicle_position.distance(preceding_position);
      reward -= PLATOONING_REWARD * Math.pow((1 / this.gap_max) * (this.gap_desired - gap), 2);

      // punish velocity difference
      double vehicle_velocity = this.velocities[vehicle_index];
      double preceding_velocity = this.velocities[preceding_index];
      reward -= PLATOONING_REWARD * Math.pow((1 / this.velocity_max) * (vehicle_velocity - preceding_velocity), 2);

      // reward below max gap
      if (this.gap_max >= gap)
        reward += PLATOONING_REWARD / 2;

      // reward below max velocity
      if (this.velocity_max >= vehicle_velocity)
        reward += PLATOONING_REWARD / 2;
    }
    else {
      // punish velocity difference
      double vehicle_velocity = this.velocities[vehicle_index];
      reward -= PLATOONING_REWARD * Math.pow((1 / this.velocity_max) * (this.velocity_desired - vehicle_velocity), 2);

      // reward below max velocity
      if (this.velocity_max >= vehicle_velocity)
        reward += PLATOONING_REWARD;
    }

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
    Stack<Vec2> vehicle_targets = this.navigations[vehicle_index].getTargets();

    int current_best_target_index = Integer.MAX_VALUE;
    Pair<Integer, Double> current_best = new Pair<>(-1, 0d);

    // Search all other vehicles for the preceding vehicle
    for (int i = 0; i < this.NUMBER_OF_VEHICLES; i++) {
      if (i == vehicle_index)
        continue;

      // keep track of the current vehicles distance to the next target
      double vehicle_distance;

      // search for matching targets between the two vehicles
      Stack<Vec2> i_targets = this.navigations[i].getTargets();
      for (int vehicle_target_index = 0; vehicle_target_index <= Math.min(vehicle_targets.size() - 1, current_best_target_index); vehicle_target_index++) {
        Vec2 vehicle_target = vehicle_targets.get(vehicle_target_index);

        if (vehicle_target_index == 0) {
          vehicle_distance = (this.positions[vehicle_index]).distance(i_targets.get(0));
        }
        else {
          vehicle_distance = vehicle_target.distance(vehicle_targets.get(vehicle_target_index - 1));
        }

        // keep track of the other vehicles distance to the next target
        double i_distance;
        for (int i_target_index = 0; i_target_index < vehicle_targets.size(); i_target_index++) {
          Vec2 i_target = i_targets.get(i_target_index);

          if (i_target_index == 0) {
            i_distance = (this.positions[i]).distance(i_targets.get(0));
          }
          else {
            i_distance = i_target.distance(i_targets.get(i_target_index - 1));
          }

          // check if the target points are equal (i.e. share the same trajectory from here on)
          if (vehicle_target.equals(i_target)) {

            double distance = vehicle_distance - i_distance;

            // the other vehicle will only precede the current vehicle if it will arrive at that point first
            if (distance > 0) {
              boolean replace = vehicle_target_index < current_best_target_index || current_best.getValue() > distance;
              // do not replace
              if (replace) {
                current_best_target_index = vehicle_target_index;
                current_best = new Pair<>(i, distance);
              }
            }
          }
        }
      }
    }

    // if we haven't found a preceding vehicle, the current vehicle is a leader
    if (current_best_target_index == Integer.MAX_VALUE) {
      return Optional.empty();
    }

    return Optional.of(current_best.getKey());
  }
}
