package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("vehicle_collision_reward")
public class VehicleCollisionsRewardFunctionProperties extends RewardFunctionProperties {

  public float VEHICLE_COLLISIONS_REWARD = -800;

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new VehicleCollisionsRewardFunction(vehicles, tickDuration, VEHICLE_COLLISIONS_REWARD);
  }
}
