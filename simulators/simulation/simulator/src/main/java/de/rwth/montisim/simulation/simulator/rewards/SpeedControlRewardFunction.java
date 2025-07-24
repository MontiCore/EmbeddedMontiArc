package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward Function that evaluates the speed and steadiness of a vehicle.
 * @see de.rwth.montisim.simulation.simulator.rewards.VariableSpeedControlRewardFunction
 */
public class SpeedControlRewardFunction extends VariableSpeedControlRewardFunction {

  /**
   * Initializes a new Speed Control Reward Function
   *
   * @param vehicles                                 Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration                             Duration between two Updates of the Simulator.
   * @param VELOCITY_DIFFERENCE_SCALING              Scaling factor of the reward given to the difference in velocity in relation to the desired velocity.
   * @param VELOCITY_SUB_MAXIMUM_REWARD              Reward given to a vehicle driving below the maximum velocity.
   * @param STANDING_PUNISHMENT                      Punishment given to a vehicle Standing still.
   * @param LINEAR_ACCELERATION_SCALING                   Scaling factor of the reward given to the control effort (acceleration).
   * @param LINEAR_JERK_SCALING        Scaling factor of the reward given to the derivative of control effort (acceleration).
   * @param velocity_max                             Maximum allowed velocity.
   * @param velocity_desired                         Desired velocity.
   * @param standing_threshold                       The upper bound of velocity of a vehicle that is considered to be standing still.
   * @param standing_punishment_from_step            From which simulation step onwards a punishment for standing still should be considered.
   * @param standing_punishment_to_min_path_distance To which distance to the goal of the vehicle a punishment for standing still should be considered.
   */
  public SpeedControlRewardFunction(Vehicle[] vehicles, Duration tickDuration, float VELOCITY_DIFFERENCE_SCALING, float VELOCITY_SUB_MAXIMUM_REWARD, float STANDING_PUNISHMENT, float LINEAR_ACCELERATION_SCALING, float LINEAR_JERK_SCALING, float ANGULAR_ACCELERATIION_SCALING, float ANGULAR_JERK_SCALING, float velocity_max, float velocity_desired, float standing_threshold, int standing_punishment_from_step, float standing_punishment_to_min_path_distance) {
    super(vehicles, tickDuration, VELOCITY_DIFFERENCE_SCALING, VELOCITY_SUB_MAXIMUM_REWARD, STANDING_PUNISHMENT, LINEAR_ACCELERATION_SCALING, LINEAR_JERK_SCALING, ANGULAR_ACCELERATIION_SCALING, ANGULAR_JERK_SCALING, velocity_max, velocity_desired, standing_threshold, standing_punishment_from_step, standing_punishment_to_min_path_distance);
  }
}
