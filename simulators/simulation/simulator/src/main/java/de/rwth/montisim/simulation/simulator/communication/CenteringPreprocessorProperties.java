package de.rwth.montisim.simulation.simulator.communication;

import de.rwth.montisim.commons.utils.json.Typed;

/**
 * Properties for the Centering Preprocessor.
 */
@Typed("centering")
public class CenteringPreprocessorProperties extends PreprocessorProperties {
  @Override
  public Preprocessor build() {
    return new CenteringPreprocessor();
  }
}
