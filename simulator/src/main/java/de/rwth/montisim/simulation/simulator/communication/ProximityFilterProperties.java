package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

/**
 * Properties for the Proximity Filter Preprocessor.
 */
@Typed("proximity")
public class ProximityFilterProperties extends PreprocessorProperties {

    public int maxNumberOfVehicles = 5;
    public boolean addIndicator = true;

    @Override
    public Preprocessor build() {
        return new ProximityFilter(maxNumberOfVehicles, addIndicator);
    }
}
