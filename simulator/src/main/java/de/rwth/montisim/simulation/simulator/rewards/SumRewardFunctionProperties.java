package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;

import java.time.Duration;
import java.util.Vector;

/**
 * The properties for the sequence reward function.
 *
 * @see SumRewardFunction
 */
@Typed("sum_reward")
public class SumRewardFunctionProperties extends RewardFunctionProperties {
  /**
   * The properties of the preprocessors that should be applied.
   */
  public Vector<RewardFunctionProperties> rewardFunctions = new Vector<>();

  public Vector<Float> rewardWeights = new Vector<>();

  @Override
  public RewardFunction build(Vehicle[] vehicles, Duration tickDuration) {
    RewardFunction[] rewardFunctionsArray = new RewardFunction[rewardFunctions.size()];
    for (int i = 0; i < rewardFunctions.size(); i++) {
      rewardFunctionsArray[i] = rewardFunctions.get(i).build(vehicles, tickDuration);
    }
    return new SumRewardFunction(vehicles, tickDuration, rewardFunctionsArray);
  }
}
