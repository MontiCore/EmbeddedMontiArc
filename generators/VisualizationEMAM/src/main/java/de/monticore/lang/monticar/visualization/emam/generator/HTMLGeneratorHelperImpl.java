/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.generator;

import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;

import java.util.logging.Logger;

public class HTMLGeneratorHelperImpl implements HTMLGeneratorHelper {
    protected final Logger logger;
    protected final OptionsService optionsService;

    @Inject
    public HTMLGeneratorHelperImpl(Logger logger, OptionsService optionsService) {
        this.logger = logger;
        this.optionsService = optionsService;
    }

    @Override
    public String getName() {
        String model = this.optionsService.getOptionAsString("m");
        String[] modelParts = model.split("\\.");
        String name = modelParts[modelParts.length - 1];

        return name.substring(0, 1).toUpperCase() + name.substring(1);
    }

    @Override
    public String getInput() {
        return this.optionsService.getOptionAsString("m");
    }
}
