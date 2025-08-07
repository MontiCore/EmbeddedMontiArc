package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

/**
 * The properties of the AnglePreprocessor.
 */
@Typed("angle")
public class AnglePreprocessorProperties extends PreprocessorProperties {

    /**
     * Index of the angle in the vehicle state
     */
    public int angleStateIndex = 23;

    /**
     * Index of the angle state packet index
     */
    public int angleStatePacketIndex = 23;

    @Override
    public Preprocessor build() {
        return new AnglePreprocessor(angleStateIndex, angleStatePacketIndex);
    }
}
