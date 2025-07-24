package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

import java.util.Vector;

/**
 * The properties for the sequence preprocessor.
 * @see de.rwth.montisim.simulation.simulator.communication.SequencePreprocessor
 */
@Typed("sequence")
public class SequencePreprocessorProperties extends PreprocessorProperties {

    /**
     * The properties of the preprocessors that should be applied.
     */
    public Vector<PreprocessorProperties> preprocessors = new Vector<>();

    @Override
    public Preprocessor build() {
        Preprocessor[] preprocessorsArray = new Preprocessor[preprocessors.size()];
        for (int i = 0; i < preprocessors.size(); i++) {
            preprocessorsArray[i] = preprocessors.get(i).build();
        }
        return new SequencePreprocessor(preprocessorsArray);
    }
}
