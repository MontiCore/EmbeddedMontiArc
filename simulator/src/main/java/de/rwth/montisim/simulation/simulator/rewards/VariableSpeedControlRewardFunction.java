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
  private final float CONTROL_EFFORT_REWARD_SCALING;
  private final float CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING;
  protected float velocity_max;
  protected float velocity_desired;
  protected float standing_threshold;
  protected int standing_punishment_from_step;
  protected float standing_punishment_to_min_path_distance;
  protected double[][] past_velocities;

  /**
   * Initializes a new Variable Speed Control Reward Function
   *
   * @param vehicles                                 Vehicle[] containing additional data about each active vehicle.
   * @param tickDuration                             Duration between two Updates of the Simulator.
   * @param VELOCITY_DIFFERENCE_REWARD_SCALING       Scaling factor of the reward given to the difference in velocity in relation to the desired velocity.
   * @param VELOCITY_SUB_MAXIMUM_REWARD              Reward given to a vehicle driving below the maximum velocity.
   * @param STANDING_REWARD                          Punishment given to a vehicle Standing still.
   * @param CONTROL_EFFORT_REWARD_SCALING            Scaling factor of the reward given to the control effort (acceleration).
   * @param CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING Scaling factor of the reward given to the derivative of control effort (acceleration).
   * @param velocity_max                             Maximum allowed velocity.
   * @param velocity_desired                         Desired velocity.
   * @param standing_threshold                       The upper bound of velocity of a vehicle that is considered to be standing still.
   * @param standing_punishment_from_step            From which simulation step onwards a punishment for standing still should be considered.
   * @param standing_punishment_to_min_path_distance To which distance to the goal of the vehicle a punishment for standing still should be considered.
   */
  public VariableSpeedControlRewardFunction(Vehicle[] vehicles, Duration tickDuration, float VELOCITY_DIFFERENCE_REWARD_SCALING, float VELOCITY_SUB_MAXIMUM_REWARD, float STANDING_REWARD, float CONTROL_EFFORT_REWARD_SCALING, float CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING, float velocity_max, float velocity_desired, float standing_threshold, int standing_punishment_from_step, float standing_punishment_to_min_path_distance) {
    super(vehicles, tickDuration);
    this.VELOCITY_DIFFERENCE_REWARD_SCALING = VELOCITY_DIFFERENCE_REWARD_SCALING;
    this.VELOCITY_SUB_MAXIMUM_REWARD = VELOCITY_SUB_MAXIMUM_REWARD;
    this.STANDING_REWARD = STANDING_REWARD;
    this.CONTROL_EFFORT_REWARD_SCALING = CONTROL_EFFORT_REWARD_SCALING;
    this.CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING = CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING;
    this.velocity_max = velocity_max;
    this.velocity_desired = velocity_desired;
    this.standing_threshold = standing_threshold;
    this.standing_punishment_from_step = standing_punishment_from_step;
    this.standing_punishment_to_min_path_distance = standing_punishment_to_min_path_distance;
    this.past_velocities = new double[NUMBER_OF_VEHICLES][2];
  }

  @Override
  public float getRewardForVehicle(int vehicle_index, int step) {
    float reward = 0;
    // punish velocity difference
    double velocity = this.velocities[vehicle_index];
    reward -= VELOCITY_DIFFERENCE_REWARD_SCALING * Math.pow((1 / this.velocity_max) * (this.velocity_desired - velocity), 2);

    // reward below max velocity
    if (this.velocity_max >= velocity)
      reward += VELOCITY_SUB_MAXIMUM_REWARD;

    // don't let the vehicle stand still
    Optional<Double> remaining_path_length = this.navigations[vehicle_index].getRemainingPathLength();
    if ((step > standing_punishment_from_step && remaining_path_length.map(dist -> dist > standing_punishment_to_min_path_distance).orElse(false)) && velocity <= standing_threshold)
      reward += STANDING_REWARD;

    // control effort (acceleration)
    double current_acceleration = (velocity - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10 ^ 9));
    reward -= CONTROL_EFFORT_REWARD_SCALING * Math.pow(current_acceleration, 2);

    // derivative of control effort
    double previous_acceleration = (past_velocities[vehicle_index][1] - past_velocities[vehicle_index][0]) / (tickDuration.getNano() / (double) (10 ^ 9));
    double derivative = (current_acceleration - previous_acceleration) / (tickDuration.getNano() / (double) (10 ^ 9));
    reward -= CONTROL_EFFORT_DERIVATIVE_REWARD_SCALING * Math.pow(derivative, 2);

    // update past velocities
    past_velocities[vehicle_index][1] = past_velocities[vehicle_index][0];
    past_velocities[vehicle_index][0] = velocity;

    return reward;
  }
}
