package de.rwth.montisim.simulation.simulator.randomization;

import de.rwth.montisim.commons.utils.json.Typed;
import de.rwth.montisim.simulation.simulator.SimulationConfig;

@Typed(EmptyStrategyProperties.TYPE_LABEL)
public class EmptyStrategyProperties extends RandomizationProperties {

    public static final String TYPE_LABEL = "empty";

    @Override
    public RandomizationStrategy build(SimulationConfig config, String mapsFolder) throws Exception {
        return new EmptyStrategy(seed);
    }

}
