package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

@Typed("static_collision_reward")
public class StaticCollisionsRewardFunctionProperties extends RewardFunctionProperties {

  public float reward = -600;

  @Override
  public RewardFunction build(Vehicle[] vehicles) {
    return new StaticCollisionsRewardFunction(vehicles, reward);
  }
}
