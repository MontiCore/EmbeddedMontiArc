/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.options;

import org.apache.commons.cli.Options;

public class OutputPathOption implements OptionsContribution {
    @Override
    public void addToOptions(Options options) {
        options.addOption("out", "outputPath", true, "Path to the output directory.");
    }
}
