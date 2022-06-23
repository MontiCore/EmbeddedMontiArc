package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

@Typed("vehicle_collision_reward")
public class VehicleCollisionsRewardFunctionProperties extends RewardFunctionProperties {

    public float reward = -800;

    @Override
    public RewardFunction build(Navigation[] navigations, Vehicle[] vehicles) {
        return new VehicleCollisionsRewardFunction(navigations, vehicles, reward);
    }
}
