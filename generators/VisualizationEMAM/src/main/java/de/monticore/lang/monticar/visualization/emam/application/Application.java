/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.application;

import com.google.inject.Guice;
import com.google.inject.Inject;
import com.google.inject.Injector;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.dependencies.DependenciesModule;
import de.monticore.lang.monticar.visualization.emam.events.EventsModule;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;
import de.monticore.lang.monticar.visualization.emam.executables.ExecutablesModule;
import de.monticore.lang.monticar.visualization.emam.generator.GeneratorModule;
import de.monticore.lang.monticar.visualization.emam.models.ModelsModule;
import de.monticore.lang.monticar.visualization.emam.options.OptionsModule;
import de.monticore.lang.monticar.visualization.emam.options.OptionsService;
import de.monticore.lang.monticar.visualization.emam.paths.PathsModule;
import de.monticore.lang.monticar.visualization.emam.resources.ResourcesModule;

import java.text.ParseException;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class Application {
    protected final Logger logger;
    protected final Set<ApplicationContribution> contributions;
    protected final OptionsService optionsService;
    protected final EventsService eventsService;

    @Inject
    public Application(Logger logger, Set<ApplicationContribution> contributions,
                       OptionsService optionsService, EventsService eventsService) {
        this.logger = logger;
        this.contributions = contributions;
        this.optionsService = optionsService;
        this.eventsService = eventsService;
    }

    public void start(String[] args) {
        try {
            this.configure();
            this.prepare();

            if (args.length == 6) this.run(args);
            else this.optionsService.printHelp();

            this.stop();
        } catch (ParseException exception) {
            this.optionsService.printHelp();
        } catch (Exception exception) {
            this.logger.severe(exception.getMessage());
            System.exit(1);
        }
    }

    protected void configure() throws Exception {
        this.logger.info("Configuring Application...");

        for (ApplicationContribution contribution : this.contributions) {
            contribution.configure(this);
        }
    }

    protected void prepare() throws Exception {
        this.logger.info("Preparing Application...");

        for (ApplicationContribution contribution : this.contributions) {
            contribution.prepare(this);
        }
    }

    protected void run(String[] args) throws Exception {
        this.logger.info("Running Application...");

        for (ApplicationContribution contribution : this.contributions) {
            contribution.start(this, args);
        }
    }

    protected void stop() throws Exception {
        this.logger.info("Stopping Application...");

        for (ApplicationContribution contribution : this.contributions) {
            contribution.stop(this);
        }
    }


    public static void main(String[] args) {
        Injector injector = Guice.createInjector(
            new ApplicationModule(), new DependenciesModule(), new ExecutablesModule(),
            new OptionsModule(), new PathsModule(), new ModelsModule(), new EventsModule(),
            new GeneratorModule(), new ResourcesModule()
        );
        Application application = injector.getInstance(Application.class);

        application.start(args);
    }
}
