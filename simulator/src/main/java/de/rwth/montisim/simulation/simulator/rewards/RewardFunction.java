package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

/**
 * Abstract class that specifies a reward function, evaluating an action taken by an agent in the environment.
 */
public abstract class RewardFunction {

  final int NUMBER_OF_VEHICLES;
  final Navigation[] navigations;
  final Vehicle[] vehicles;

  /**
   * Default constructor that initializes the parameters required for any given reward function: Navigation and Data of each Vehicle.
   *
   * @param navigations Navigation[] containing the currently active vehicle's navigation.
   * @param vehicles    Vehicle[] containing additional data about each active vehicle.
   */
  public RewardFunction(Navigation[] navigations, Vehicle[] vehicles) {
    assert navigations.length == vehicles.length;

    this.NUMBER_OF_VEHICLES = navigations.length;
    this.navigations = navigations;
    this.vehicles = vehicles;
  }

  /**
   * Evaluates the current environment as a whole and calculates the reward score for it.
   * The default implementation returns the cumulated reward of each vehicle {@link #getRewardForVehicle(int)}.
   *
   * @return the reward for the current environment.
   */
  public float getReward() {
    float reward = 0;
    for (int i = 0; i < NUMBER_OF_VEHICLES; i++) {
      reward += getRewardForVehicle(i);
    }
    return reward;
  }

  /**
   * Evaluates the current environment from the perspective of a single vehicle and calculates the reward score for it.
   *
   * @param vehicle_index Index of the vehicle to calculate the reward for.
   * @return the reward for the current environment from the specified vehicle's perspective.
   */
  public abstract float getRewardForVehicle(int vehicle_index);

}
