/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.options;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;
import org.apache.commons.cli.*;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class OptionsServiceImpl implements OptionsService, ApplicationContribution {
    protected final Logger logger;
    protected final Set<OptionsContribution> contributions;
    protected final EventsService eventsService;
    protected final Options options;

    protected CommandLine commandLine;

    @Inject
    public OptionsServiceImpl(Logger logger, Set<OptionsContribution> contributions, EventsService eventsService) {
        this.options = new Options();
        this.logger = logger;
        this.contributions = contributions;
        this.eventsService = eventsService;
    }

    @Override
    public void configure(Application application) {
        this.logger.info("Adding Options to Command Line Interface...");

        for (OptionsContribution contribution : this.contributions) {
            contribution.addToOptions(this.options);
        }
    }

    @Override
    public void start(Application application, String[] args) throws Exception {
        CommandLineParser parser = new DefaultParser();

        this.commandLine = parser.parse(this.options, args);

        this.eventsService.post(new OptionsParsedEvent(this));
    }

    @Override
    public void printHelp() {
        HelpFormatter formatter = new HelpFormatter();

        formatter.printHelp("VisualizationEMAM", this.options);
    }

    @Override
    public String getOptionAsString(String option) {
        return this.commandLine.getOptionValue(option);
    }

    @Override
    public Path getOptionAsPath(String option) {
        String optionString = this.getOptionAsString(option);

        return Paths.get(optionString).toAbsolutePath();
    }

    @Override
    public File getOptionAsFile(String option) {
        return this.getOptionAsPath(option).toFile();
    }
}
