package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("static_collision_reward")
public class StaticCollisionsRewardFunctionProperties extends RewardFunctionProperties {

  public float STATIC_COLLISION_REWARD = -600;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new StaticCollisionsRewardFunction(vehicles, tickDuration, STATIC_COLLISION_REWARD);
  }
}
