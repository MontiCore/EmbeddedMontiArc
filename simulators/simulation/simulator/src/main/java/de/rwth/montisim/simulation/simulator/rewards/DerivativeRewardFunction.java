package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

/**
 * Reward function that punishes high amounts of acceleration or jerk
 */
public class DerivativeRewardFunction extends RewardFunction {

    private final float ANGULAR_JERK_REWARD_FACTOR;

    private final float ANGULAR_ACC_REWARD_FACTOR;

    private final float LINEAR_JERK_REWARD_FACTOR;

    private final float LINEAR_ACC_REWARD_FACTOR;

    private final boolean QUADRATIC;

    private double pastLinearVelocity = 0;

    private double pastLinearAcceleration = 0;

    private double pastAngle = 0;

    private double pastAngularVelocity = 0;

    private double pastAngularAcceleration = 0;



    public DerivativeRewardFunction(Vehicle[] vehicles, Duration tickDuration, float angular_jerk_reward_factor,
                                    float angular_acc_reward_factor, float linear_jerk_reward_factor, float linear_acc_reward_factor,
                                    boolean quadratic) {
        super(vehicles, tickDuration);
        ANGULAR_JERK_REWARD_FACTOR = angular_jerk_reward_factor;
        ANGULAR_ACC_REWARD_FACTOR = angular_acc_reward_factor;
        LINEAR_JERK_REWARD_FACTOR = linear_jerk_reward_factor;
        LINEAR_ACC_REWARD_FACTOR = linear_acc_reward_factor;
        QUADRATIC = quadratic;
    }

    @Override
    public float getRewardForVehicle(int vehicle_index, int step) {
        // Compute Linear Derivatives
        double linearVelocity = (Double) velocities[vehicle_index].get();
        double linearAcceleration = (linearVelocity - pastLinearVelocity) / (tickDuration.getNano() / (double) (10 ^ 9));
        double linearJerk = (linearAcceleration - pastLinearAcceleration) / (tickDuration.getNano() / (double) (10 ^ 9));

        // Compute Angular Derivatives in rad per second
        double angle = ((Double) angles[vehicle_index].get()) / 180 * Math.PI;
        double angularVelocity = (angle - pastAngle) / (tickDuration.getNano() / (double) (10 ^ 9));
        double angularAcceleration = (angularVelocity - pastAngularVelocity) / (tickDuration.getNano() / (double) (10 ^ 9));
        double angularJerk = (angularAcceleration - pastAngularAcceleration) / (tickDuration.getNano() / (double) (10 ^ 9));

        // Set past values
        pastLinearVelocity = linearVelocity;
        pastLinearAcceleration = linearAcceleration;
        pastAngle = angle;
        pastAngularVelocity = angularVelocity;
        pastAngularAcceleration = angularAcceleration;

        // Compute Reward
        float reward = (QUADRATIC)? (float) (
                    - LINEAR_ACC_REWARD_FACTOR * Math.pow(linearAcceleration, 2)
                    - LINEAR_JERK_REWARD_FACTOR * Math.pow(linearJerk, 2)
                    - ANGULAR_ACC_REWARD_FACTOR * Math.pow(angularAcceleration, 2)
                    - ANGULAR_JERK_REWARD_FACTOR * Math.pow(angularJerk, 2))
                : (float) (
                    - LINEAR_ACC_REWARD_FACTOR * Math.abs(linearAcceleration)
                    - LINEAR_JERK_REWARD_FACTOR * Math.abs(linearJerk)
                    - ANGULAR_ACC_REWARD_FACTOR * Math.abs(angularAcceleration)
                    - ANGULAR_JERK_REWARD_FACTOR * Math.abs(angularJerk));

        return reward;
    }
}
