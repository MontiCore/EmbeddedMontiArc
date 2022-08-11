package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Arrays;

/**
 * Reward function that calculates the reward by applying a sequence of other reward functions.
 */
public class SumRewardFunction extends RewardFunction {

  private final RewardFunction[] components;
  private final float[] weights;


  /**
   * Initializes the sum reward function.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   * @param components   The sum of reward functions that should be applied.
   */
  public SumRewardFunction(Vehicle[] vehicles, Duration tickDuration, RewardFunction[] components) {
    super(vehicles, tickDuration);
    this.components = components;
    weights = new float[components.length];
    Arrays.fill(weights, 1f);
  }

  /**
   * Initializes the sum reward function.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   * @param components   The sum of reward functions that should be applied.
   */
  public SumRewardFunction(Vehicle[] vehicles, Duration tickDuration, RewardFunction[] components, float[] weights) {
    super(vehicles, tickDuration);
    this.components = components;
    this.weights = weights;
  }

  /**
   * Initializes the sum reward function.
   *
   * @param vehicles     Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration Duration between two Updates of the Simulator.
   */
  public SumRewardFunction(Vehicle[] vehicles, Duration tickDuration) {
    super(vehicles, tickDuration);
    this.components = new RewardFunction[0];
    weights = new float[0];
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;
    for (int i = 0; i < components.length; i++) {
      reward += components[i].getRewardForVehicle(vehicle_index, step) * weights[i];
    }
    return reward;
  }
}
