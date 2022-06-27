package de.rwth.montisim.simulation.simulator.rewards;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.vehicle.Vehicle;
import de.rwth.montisim.simulation.vehicle.navigation.Navigation;

import java.util.Vector;

/**
 * The properties for the sequence reward function.
 *
 * @see de.rwth.montisim.simulation.simulator.rewards.SequenceRewardFunction
 */
@Typed("sequence_reward")
public class SequenceRewardFunctionProperties extends RewardFunctionProperties {
  /**
   * The properties of the preprocessors that should be applied.
   */
  public Vector<RewardFunctionProperties> rewardFunctions = new Vector<>();

  @Override
  public RewardFunction build(Navigation[] navigations, Vehicle[] vehicles) {
    RewardFunction[] rewardFunctionsArray = new RewardFunction[rewardFunctions.size()];
    for (int i = 0; i < rewardFunctions.size(); i++) {
      rewardFunctionsArray[i] = rewardFunctions.get(i).build(navigations, vehicles);
    }
    return new SequenceRewardFunction(navigations, vehicles, rewardFunctionsArray);
  }
}
