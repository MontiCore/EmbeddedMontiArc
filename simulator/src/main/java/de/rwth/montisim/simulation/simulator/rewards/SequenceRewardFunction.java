package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

/**
 * Reward function that calculates the reward by applying a sequence of other reward functions.
 */
public class SequenceRewardFunction extends RewardFunction {

  private final RewardFunction[] components;

  /**
   * Initializes the sequence reward function.
   *
   * @param vehicles    Vehicle[] containing additional data about each active vehicle.
   * @param components  The sequence of reward functions that should be applied.
   */
  public SequenceRewardFunction(Vehicle[] vehicles, RewardFunction[] components) {
    super(vehicles);
    this.components = components;
  }

  /**
   * Initializes the sequence reward function.
   *
   * @param vehicles    Vehicle[] containing additional data about each active vehicle.
   */
  public SequenceRewardFunction(Vehicle[] vehicles) {
    super(vehicles);
    this.components = new RewardFunction[0];
  }

  @Override
  public float getRewardForVehicle(int vehicle_index) {
    float reward = 0;
    for (int i = 0; i < components.length; i++) {
      reward += components[i].getRewardForVehicle(vehicle_index);
    }
    return reward;
  }
}
