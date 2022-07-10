package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward Function that evaluates the speed and steadiness of a vehicle.
 */
public class SpeedControlRewardFunction extends RewardFunction {

  private final float SPEED_CONTROL_REWARD;
  private final float velocity_max;
  private final float velocity_desired;

  private final double[][] past_velocities;

  /**
   * Initializes a new Speed Control Reward Function
   *
   * @param vehicles             Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration         Duration between two Updates of the Simulator.
   * @param speed_control_reward The scaled reward.
   * @param velocity_max         Maximum allowed velocity.
   * @param velocity_desired     Desired velocity.
   */
  public SpeedControlRewardFunction(Vehicle[] vehicles, Duration tickDuration, float speed_control_reward, float velocity_max, float velocity_desired) {
    super(vehicles, tickDuration);
    this.SPEED_CONTROL_REWARD = speed_control_reward;
    this.velocity_max = velocity_max;
    this.velocity_desired = velocity_desired;
    this.past_velocities = new double[NUMBER_OF_VEHICLES][2];
  }

  @Override
  public float getRewardForVehicle(int vehicle_index) {
    float reward = 0;
    // punish velocity difference
    double velocity = this.velocities[vehicle_index];
    reward -= Math.pow((1 / this.velocity_max) * (this.velocity_desired - velocity), 2);

    // reward below max velocity
    if (this.velocity_max >= velocity)
      reward += 0.1;

    // don't let the vehicle stant still
    if (velocity <= 0.001)
      reward -= 10;

    // control effort (acceleration)
    double current_acceleration = (velocity - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10^9));
    reward -= Math.pow(current_acceleration, 2);

    // derivative of control effort
    double previous_acceleration = (past_velocities[vehicle_index][1] - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10^9));
    double derivative = (current_acceleration - previous_acceleration) / (tickDuration.getNano() / (double) (10^9));
    reward -= 0.05 * Math.pow(derivative, 2);

    // update past velocities
    past_velocities[vehicle_index][1] = past_velocities[vehicle_index][0];
    past_velocities[vehicle_index][0] = velocity;

    return reward * SPEED_CONTROL_REWARD;
  }
}
