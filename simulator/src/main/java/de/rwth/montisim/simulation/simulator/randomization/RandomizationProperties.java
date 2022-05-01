package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Optional;

import de.rwth.montisim.simulation.simulator.SimulationConfig;

/**
 * Abstract class defining the necessary properties of every randomization
 * strategy and defining an abstract method for building its strategy.
 * 
 * @see de.rwth.montisim.simulation.simulator.randomization.RandomizationStrategy
 */
public abstract class RandomizationProperties {
  
  /**
   * The seed used for the randomization
   */
  public Optional<Long> seed = Optional.empty();
  
  /**
   * Builds a RandomizationStrategy that randomizes a given scenario
   * 
   * @param config The configuration object of the scenario
   * @param mapsFolder A path to the folder where the map files are stored in
   * @return A RandomizationStrategy that can be used to randomize aspects of
   *         the scenario
   * @throws Exception
   */
  public abstract RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception;
  
}