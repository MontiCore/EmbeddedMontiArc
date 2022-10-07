package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward function that punishes vehicle collisions.
 */
public class VehicleCollisionsRewardFunction extends RewardFunction {

  private final float VEHICLE_COLLISION_REWARD;

  /**
   * Reward function that punishes vehicle collisions.
   *
   * @param vehicles                  Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration              Duration between two Updates of the Simulator.
   * @param VEHICLE_COLLISIONS_REWARD The reward given for each vehicle collision.
   */
  public VehicleCollisionsRewardFunction(Vehicle[] vehicles, Duration tickDuration, float VEHICLE_COLLISIONS_REWARD) {
    super(vehicles, tickDuration);
    VEHICLE_COLLISION_REWARD = VEHICLE_COLLISIONS_REWARD;
  }

  @Override
  public float getReward(int step) {
        /*
         If a vehicle collides with another, the reward would be given twice. Thus, we need to halve this reward.
         */
    return super.getReward(step) / 2;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    int numberOfCollisions = this.vehicles[vehicle_index].getVehicleCollisions().size();
    return numberOfCollisions * VEHICLE_COLLISION_REWARD;
  }
}
