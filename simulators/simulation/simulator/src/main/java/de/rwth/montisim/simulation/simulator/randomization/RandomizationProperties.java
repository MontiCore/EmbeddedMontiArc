package de.rwth.montisim.simulation.simulator.randomization;

import java.util.Optional;

import de.rwth.montisim.simulation.simulator.SimulationConfig;

public abstract class RandomizationProperties {

    public Optional<Long> seed = Optional.empty();

    public abstract RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception;

}
