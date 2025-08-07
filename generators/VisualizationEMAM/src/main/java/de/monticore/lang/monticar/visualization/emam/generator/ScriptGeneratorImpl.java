/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;

import java.io.File;
import java.util.logging.Logger;

@Singleton
public class ScriptGeneratorImpl implements ScriptGenerator {
    protected final Logger logger;
    protected final ScriptGeneratorHelper helper;
    protected final OptionsService optionsService;

    protected GeneratorEngine generator;

    @Inject
    public ScriptGeneratorImpl(Logger logger, ScriptGeneratorHelper helper, OptionsService optionsService) {
        this.logger = logger;
        this.helper = helper;
        this.optionsService = optionsService;
    }

    @Override
    public String generate(String packige, String name, String body) {
        GeneratorSetup setup = new GeneratorSetup();
        File outputDirectory = this.optionsService.getOptionAsFile("mp");
        GlobalExtensionManagement glex = new GlobalExtensionManagement();

        glex.setGlobalValue("helper", this.helper);

        setup.setGlex(glex);
        setup.setTracing(false);
        setup.setOutputDirectory(outputDirectory);

        this.generator = new GeneratorEngine(setup);

        this.helper.setPackage(packige);
        this.helper.setName(name);
        this.helper.setBody(body);

        return this.generator.generateNoA("templates/script.ftl").toString();
    }
}
