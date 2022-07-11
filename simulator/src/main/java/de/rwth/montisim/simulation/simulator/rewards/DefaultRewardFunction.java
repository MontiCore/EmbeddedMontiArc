package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

public class DefaultRewardFunction extends RewardFunction {

  public DefaultRewardFunction(Vehicle[] vehicles, Duration tickDuration) {
    super(vehicles, tickDuration);
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    return 0;
  }
}
