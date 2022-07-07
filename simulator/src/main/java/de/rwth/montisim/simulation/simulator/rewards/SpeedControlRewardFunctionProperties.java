package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

public class SpeedControlRewardFunctionProperties extends RewardFunctionProperties {

  public float speed_control_reward = 1;
  public float velocity_max = 40;
  public float velocity_desired = 20;
  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    return new SpeedControlRewardFunction(vehicles, tickDuration, speed_control_reward, velocity_max, velocity_desired);
  }
}
