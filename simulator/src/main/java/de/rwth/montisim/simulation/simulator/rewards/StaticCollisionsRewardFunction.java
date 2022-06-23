package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

/**
 * Reward function that punishes static collisions.
 */
public class StaticCollisionsRewardFunction extends RewardFunction {

    private final float STATIC_COLLISION_REWARD;

    /**
     * Reward function that punishes static collisions.
     *
     * @param navigations             Navigation[] containing the currently active vehicle's navigation.
     * @param vehicles                Vehicle[] containing additional data about each active vehicle.
     * @param static_collision_reward The reward given for each static collision.
     */
    public StaticCollisionsRewardFunction(Navigation[] navigations, Vehicle[] vehicles, float static_collision_reward) {
        super(navigations, vehicles);
        STATIC_COLLISION_REWARD = static_collision_reward;
    }

    @Override
    public float getRewardForVehicle(int vehicle_index) {
        int numberOfCollisions = this.vehicles[vehicle_index].getStaticCollisions().size();
        return numberOfCollisions * STATIC_COLLISION_REWARD;
    }
}
