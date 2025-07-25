package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("default_reward")
public class DefaultRewardFunctionProperties extends RewardFunctionProperties {

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new DefaultRewardFunction(vehicles, tickDuration);
  }
}
