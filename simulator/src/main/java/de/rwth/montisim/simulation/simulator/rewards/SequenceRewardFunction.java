package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward function that calculates the reward by applying a sequence of other reward functions.
 */
public class SequenceRewardFunction extends RewardFunction {

  private final RewardFunction[] components;

  /**
   * Initializes the sequence reward function.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   * @param components   The sequence of reward functions that should be applied.
   */
  public SequenceRewardFunction(Vehicle[] vehicles, Duration tickDuration, RewardFunction[] components) {
    super(vehicles, tickDuration);
    this.components = components;
  }

  /**
   * Initializes the sequence reward function.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   */
  public SequenceRewardFunction(Vehicle[] vehicles, Duration tickDuration) {
    super(vehicles, tickDuration);
    this.components = new RewardFunction[0];
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;
    for (int i = 0; i < components.length; i++) {
      reward += components[i].getRewardForVehicle(vehicle_index, step);
    }
    return reward;
  }
}
