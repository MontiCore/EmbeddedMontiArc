package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

/**
 * Properties class for the default preprocessor.
 * The preprocessor has the type 'default' and no parameters.
 */
@Typed("default")
public class DefaultPreprocessorProperties extends PreprocessorProperties {

    @Override
    public Preprocessor build() {
        return new DefaultPreprocessor();
    }
}
