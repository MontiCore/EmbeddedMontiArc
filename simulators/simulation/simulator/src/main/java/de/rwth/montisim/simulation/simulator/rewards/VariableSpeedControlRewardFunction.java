package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Optional;

/**
 * Reward Function that evaluates the speed and steadiness of a vehicle.
 * It allows changing some attributes at runtime, necessary for some reward functions such as {@link PlatooningRewardFunction}
 * This class is meant to be used by other reward functions only, hiding away the non-final attributes.
 */
class VariableSpeedControlRewardFunction extends RewardFunction {

  private final float VELOCITY_DIFFERENCE_REWARD_SCALING;
  private final float VELOCITY_SUB_MAXIMUM_REWARD;
  private final float STANDING_REWARD;
  private final float LINEAR_ACCELERATION_REWARD_SCALING;
  private final float LINEAR_JERK_REWARD_SCALING;
  private final float ANGULAR_ACCELERATION_REWARD_SCALING;
  private final float ANGULAR_JERK_REWARD_SCALING;
  private final DerivativeRewardFunction derivativeRewardFunction;
  protected float velocity_max;
  protected float velocity_desired;
  protected float standing_threshold;
  protected int standing_punishment_from_step;
  protected float standing_punishment_to_min_path_distance;
  // protected double[][] past_velocities;

  /**
   * Initializes a new Variable Speed Control Reward Function
   *
   * @param vehicles                                 Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration                             Duration between two Updates of the Simulator.
   * @param VELOCITY_DIFFERENCE_REWARD_SCALING       Scaling factor of the reward given to the difference in velocity in relation to the desired velocity.
   * @param VELOCITY_SUB_MAXIMUM_REWARD              Reward given to a vehicle driving below the maximum velocity.
   * @param STANDING_REWARD                          Punishment given to a vehicle Standing still.
   * @param LINEAR_ACCELERATION_REWARD_SCALING            Scaling factor of the reward given to the control effort (acceleration).
   * @param LINEAR_JERK_REWARD_SCALING Scaling factor of the reward given to the derivative of control effort (acceleration).
   * @param velocity_max                             Maximum allowed velocity.
   * @param velocity_desired                         Desired velocity.
   * @param standing_threshold                       The upper bound of velocity of a vehicle that is considered to be standing still.
   * @param standing_punishment_from_step            From which simulation step onwards a punishment for standing still should be considered.
   * @param standing_punishment_to_min_path_distance To which distance to the goal of the vehicle a punishment for standing still should be considered.
   */
  public VariableSpeedControlRewardFunction(Vehicle[] vehicles, Duration tickDuration, float VELOCITY_DIFFERENCE_REWARD_SCALING, float VELOCITY_SUB_MAXIMUM_REWARD, float STANDING_REWARD, float LINEAR_ACCELERATION_REWARD_SCALING, float LINEAR_JERK_REWARD_SCALING, float ANGULAR_ACCELERATION_REWARD_SCALING, float ANGULAR_JERK_REWARD_SCALING, float velocity_max, float velocity_desired, float standing_threshold, int standing_punishment_from_step, float standing_punishment_to_min_path_distance) {
    super(vehicles, tickDuration);
    this.VELOCITY_DIFFERENCE_REWARD_SCALING = VELOCITY_DIFFERENCE_REWARD_SCALING;
    this.VELOCITY_SUB_MAXIMUM_REWARD = VELOCITY_SUB_MAXIMUM_REWARD;
    this.STANDING_REWARD = STANDING_REWARD;
    this.LINEAR_ACCELERATION_REWARD_SCALING = LINEAR_ACCELERATION_REWARD_SCALING;
    this.LINEAR_JERK_REWARD_SCALING = LINEAR_JERK_REWARD_SCALING;
    this.ANGULAR_ACCELERATION_REWARD_SCALING = ANGULAR_ACCELERATION_REWARD_SCALING;
    this.ANGULAR_JERK_REWARD_SCALING = ANGULAR_JERK_REWARD_SCALING;
    this.velocity_max = velocity_max;
    this.velocity_desired = velocity_desired;
    this.standing_threshold = standing_threshold;
    this.standing_punishment_from_step = standing_punishment_from_step;
    this.standing_punishment_to_min_path_distance = standing_punishment_to_min_path_distance;
    // this.past_velocities = new double[NUMBER_OF_VEHICLES][2];
    this.derivativeRewardFunction = new DerivativeRewardFunction(vehicles, tickDuration, ANGULAR_JERK_REWARD_SCALING,
            ANGULAR_ACCELERATION_REWARD_SCALING, LINEAR_JERK_REWARD_SCALING,
            LINEAR_ACCELERATION_REWARD_SCALING, true);
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;
    // punish velocity difference
    double velocity = (Double) this.velocities[vehicle_index].get();
    reward -= VELOCITY_DIFFERENCE_REWARD_SCALING * Math.pow((1 / this.velocity_max) * (this.velocity_desired - velocity), 2);

    // reward below max velocity
    if (this.velocity_max >= velocity && velocity > standing_threshold)
      reward += VELOCITY_SUB_MAXIMUM_REWARD;

    // don't let the vehicle stand still
    Optional<Double> remaining_path_length = this.navigations[vehicle_index].getRemainingPathLength();
    if ((step > standing_punishment_from_step && remaining_path_length.map(dist -> dist > standing_punishment_to_min_path_distance).orElse(false)) && velocity <= standing_threshold)
      reward += STANDING_REWARD;

    /*
    // control effort (acceleration)
    double current_acceleration = (velocity - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10 ^ 9));
    reward -= LINEAR_ACCELERATION_REWARD_SCALING * Math.pow(current_acceleration, 2);

    // derivative of control effort
    double previous_acceleration = (past_velocities[vehicle_index][1] - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10 ^ 9));
    double derivative = (current_acceleration - previous_acceleration) / (tickDuration.getNano() / (double) (10 ^ 9));
    reward -= LINEAR_JERK_REWARD_SCALING * Math.pow(derivative, 2);

    // update past velocities
    past_velocities[vehicle_index][1] = past_velocities[vehicle_index][0];
    past_velocities[vehicle_index][0] = velocity;
    */

    // derivative reward
    reward += derivativeRewardFunction.getRewardForVehicle(vehicle_index, step);

    return reward;
  }
}
