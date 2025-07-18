package de.rwth.montisim.simulation.simulator.randomization;

import de.rwth.montisim.simulation.eecomponents.simple_network.ModuleProperties;
import de.rwth.montisim.simulation.simulator.communication.PreprocessorProperties;
import de.rwth.montisim.simulation.simulator.rewards.RewardFunctionProperties;
import de.rwth.montisim.simulation.vehicle.VehicleProperties;

import java.time.Duration;
import java.util.Optional;
import java.util.Random;
import java.util.Vector;

/**
 * Abstract describing the different properties of a SimulationConfig that a
 * RandomizationStrategy can randomize.
 *
 * @see de.rwth.montisim.simulation.simulator.SimulationConfig
 */
public abstract class RandomizationStrategy {

  Random random;

  public RandomizationStrategy(Optional<Long> seed) {
    random = seed.map(value -> new Random(value)).orElse(new Random());
  }

  /**
   * Randomizes the map name. Can be overridden to randomly select a map at the
   * start of a simulation. The default implementation returns the name of the
   * map that was specified in the configuration.
   *
   * @param map_name The name of the map that was specified in the
   *                 configuration.
   * @return A String containing the name of the map that a simulation should
   * load.
   */
  public String randomizeMapName(String map_name) {
    return map_name;
  }

  /**
   * Randomizes the maximum duration of a simulation. Can be overridden to
   * simulate for a random amount of time. The default implementation returns
   * the duration that was specified in the configuration.
   *
   * @param max_duration The maximum duration that was specified in the
   *                     configuration.
   * @return The maximum Duration that a simulation should run.
   */
  public Duration randomizeMaxDuration(Duration max_duration) {
    return max_duration;
  }

  /**
   * Randomizes the tick duration of a simulation. Can be overridden to simulate
   * with a randomized tick duration. The default implementation returns the
   * duration that was specified in the configuration.
   *
   * @param tick_duration The tick duration that was specified in the
   *                      configuration.
   * @return The Duration of a tick in a simulation.
   */
  public Duration randomizeTickDuration(Duration tick_duration) {
    return tick_duration;
  }

  /**
   * Randomizes the properties of the vehicles in a simulation. Can be
   * overridden to simulate randomized vehicles. This includes randomizing the
   * number of simulated vehicles, their starting positions, and their goals.
   *
   * @param vehicles The Vector of VehicleProperties that were specified in the
   *                 configuration.
   * @return A Vector of VehicleProperties describing what vehicles should be
   * simulated in a simulation.
   * @see de.rwth.montisim.simulation.vehicle.VehicleProperties
   */
  public Vector<VehicleProperties> randomizeCars(Vector<VehicleProperties> vehicles) {
    return vehicles;
  }

  /**
   * Randomizes the modules of a simulation.
   *
   * @param modules The Vector of ModuleProperties that were specified in the
   *                configuration.
   * @return A Vector of ModuleProperties describing what modules should be
   * simulated in a simulation.
   * @see de.rwth.montisim.simulation.eecomponents.simple_network.ModuleProperties
   */
  public Vector<ModuleProperties> randomizeModules(Vector<ModuleProperties> modules) {
    return modules;
  }

  /**
   * Randomizes the preprocessor of a simulation.
   *
   * @param preprocessor The preprocessor of the simulation that was specified
   *                     in the configuration.
   * @return A preprocessor that preprocesses the states of the vehicles in
   * the simulation.
   * @see de.rwth.montisim.simulation.simulator.communication.PreprocessorProperties;
   */
  public Optional<PreprocessorProperties> randomizePreprocessor(Optional<PreprocessorProperties> preprocessor) {
    return preprocessor;
  }

  /**
   * Randomizes the reward function for training.
   *
   * @param rewardFunction The reward function for training that was specified
   *                       in the configuration.
   * @return A reward Function that scores the current state of the simulation.
   * @see de.rwth.montisim.simulation.simulator.rewards.RewardFunctionProperties;
   */
  public Optional<RewardFunctionProperties> randomizeRewardFunction(Optional<RewardFunctionProperties> rewardFunction) {
    return rewardFunction;
  }

}