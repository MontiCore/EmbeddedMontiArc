package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;

@Typed("derivative_reward")
public class DerivativeRewardFunctionProperties extends RewardFunctionProperties {

    public float angular_jerk_reward_factor = 10;

    public float angular_acc_reward_factor = 10;

    public float linear_jerk_reward_factor = 10;

    public float linear_acc_reward_factor = 10;

    public boolean quadratic = false;

    @Override
    public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
        return new DerivativeRewardFunction(vehicles, tickDuration, angular_jerk_reward_factor, angular_acc_reward_factor,
                linear_jerk_reward_factor, linear_acc_reward_factor, quadratic);
    }
}
