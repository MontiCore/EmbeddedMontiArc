package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward function that punishes static collisions.
 */
public class StaticCollisionsRewardFunction extends RewardFunction {

  private final float STATIC_COLLISION_REWARD;

  /**
   * Reward function that punishes static collisions.
   *
   * @param vehicles                Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration            Duration between two Updates of the Simulator.
   * @param STATIC_COLLISION_REWARD The reward given for each static collision.
   */
  public StaticCollisionsRewardFunction(Vehicle[] vehicles, Duration tickDuration, float STATIC_COLLISION_REWARD) {
    super(vehicles, tickDuration);
    this.STATIC_COLLISION_REWARD = STATIC_COLLISION_REWARD;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    int numberOfCollisions = this.vehicles[vehicle_index].getStaticCollisions().size();
    return numberOfCollisions * this.STATIC_COLLISION_REWARD;
  }
}
