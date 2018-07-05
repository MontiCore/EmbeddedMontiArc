package de.monticore.lang.monticar.visualization.emam.options;

import org.apache.commons.cli.Options;

public class ModelPathOption implements OptionsContribution {
    @Override
    public void addToOptions(Options options) {
        options.addOption("mp", "modelPath", true, "Absolute Path to the root of the models.");
    }
}
