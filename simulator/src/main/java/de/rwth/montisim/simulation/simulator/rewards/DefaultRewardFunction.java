package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

public class DefaultRewardFunction extends RewardFunction {

  public DefaultRewardFunction(Vehicle[] vehicles) {
    super(vehicles);
  }

  @Override
  public float getRewardForVehicle(int vehicle_index) {
    return 0;
  }
}
