package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("old_reward")
public class OldRewardFunctionProperties extends RewardFunctionProperties {

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new OldRewardFunction(vehicles, tickDuration);
  }
}
