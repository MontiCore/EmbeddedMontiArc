/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.options;

import org.apache.commons.cli.Options;

public class ModelOption implements OptionsContribution {
    @Override
    public void addToOptions(Options options) {
        options.addOption("m", "model", true, "The Main Component for the Visualization.");
    }
}
