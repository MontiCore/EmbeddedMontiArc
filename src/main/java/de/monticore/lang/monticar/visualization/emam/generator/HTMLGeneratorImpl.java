/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.generator;

import com.google.common.eventbus.Subscribe;
import com.google.inject.Inject;
import de.monticore.generating.GeneratorEngine;
import de.monticore.generating.GeneratorSetup;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;
import de.monticore.lang.monticar.visualization.emam.executables.ExecutablesExecutedEvent;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.Logger;

public class HTMLGeneratorImpl implements HTMLGenerator, ApplicationContribution {
    protected final Logger logger;
    protected final EventsService eventsService;
    protected final OptionsService optionsService;
    protected final HTMLGeneratorHelper helper;

    @Inject
    public HTMLGeneratorImpl(Logger logger, EventsService eventsService,
                             OptionsService optionsService, HTMLGeneratorHelper helper) {
        this.logger = logger;
        this.eventsService = eventsService;
        this.optionsService = optionsService;
        this.helper = helper;
    }

    @Override
    public void generate() {
        GeneratorSetup setup = new GeneratorSetup();
        File outputDirectory = this.optionsService.getOptionAsFile("out");
        GlobalExtensionManagement glex = new GlobalExtensionManagement();
        Path outputFile = Paths.get("index.html");
        GeneratorEngine generator = new GeneratorEngine(setup);

        glex.setGlobalValue("helper", this.helper);

        setup.setGlex(glex);
        setup.setTracing(false);
        setup.setOutputDirectory(outputDirectory);

        generator.generateNoA("templates/html.ftl", outputFile);
    }

    @Override
    public void prepare(Application application) {
        this.eventsService.register(this);
    }

    @Subscribe
    public void onExecutablesExecuted(ExecutablesExecutedEvent event) {
        this.logger.info("Generating \"index.html\"...");
        this.generate();
        this.logger.info("...\"index.html\" has been generated.");
    }
}
