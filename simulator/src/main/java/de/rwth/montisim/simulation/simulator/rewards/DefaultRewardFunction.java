package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

public class DefaultRewardFunction extends RewardFunction {

    public DefaultRewardFunction(Navigation[] navigations, Vehicle[] vehicles) {
        super(navigations, vehicles);
    }

    @Override
    public float getRewardForVehicle(int vehicle_index) {
        return 0;
    }
}
