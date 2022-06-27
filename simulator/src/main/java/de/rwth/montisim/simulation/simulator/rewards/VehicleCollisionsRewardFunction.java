package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

/**
 * Reward function that punishes vehicle collisions.
 */
public class VehicleCollisionsRewardFunction extends RewardFunction {

  private final float VEHICLE_COLLISION_REWARD;

  /**
   * Reward function that punishes vehicle collisions.
   *
   * @param navigations              Navigation[] containing the currently active vehicle's navigation.
   * @param vehicles                 Vehicle[] containing additional data about each active vehicle.
   * @param vehicle_collision_reward The reward given for each vehicle collision.
   */
  public VehicleCollisionsRewardFunction(Navigation[] navigations, Vehicle[] vehicles, float vehicle_collision_reward) {
    super(navigations, vehicles);
    VEHICLE_COLLISION_REWARD = vehicle_collision_reward;
  }

  @Override
  public float getReward() {
        /*
         If a vehicle collides with another, the reward would be given twice. Thus, we need to halve this reward.
         */
    return super.getReward() / 2;
  }

  @Override
  public float getRewardForVehicle(int vehicle_index) {
    int numberOfCollisions = this.vehicles[vehicle_index].getVehicleCollisions().size();
    return numberOfCollisions * VEHICLE_COLLISION_REWARD;
  }
}
