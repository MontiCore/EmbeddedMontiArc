package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

@Typed("default_reward")
public class DefaultRewardFunctionProperties extends RewardFunctionProperties {

    @Override
    public RewardFunction build(Navigation[] navigations, Vehicle[] vehicles) {
        return new DefaultRewardFunction(navigations, vehicles);
    }
}
