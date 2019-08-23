/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.common.eventbus.Subscribe;
import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;
import de.monticore.lang.monticar.visualization.emam.models.ModelPathVisitedEvent;
import org.apache.commons.cli.ParseException;

import java.io.IOException;
import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class ExecutablesServiceImpl implements ExecutablesService, ApplicationContribution {
    protected final Logger logger;
    protected final EventsService eventsService;
    protected final Set<ExecutablesContribution> contributions;

    @Inject
    public ExecutablesServiceImpl(Logger logger, EventsService eventsService,
                                  Set<ExecutablesContribution> contributions) {
        this.logger = logger;
        this.eventsService = eventsService;
        this.contributions = contributions;
    }

    @Override
    public void prepare(Application application) {
        this.eventsService.register(this);
    }

    @Override
    public void prepare() {
        for (ExecutablesContribution contribution : this.contributions) {
            contribution.prepare();
        }
    }

    @Override
    public void execute() throws IOException {
        for (ExecutablesContribution contribution : this.contributions) {
            contribution.execute();
        }
    }

    @Subscribe
    public void onModelPathVisited(ModelPathVisitedEvent event) throws ParseException, IOException {
        this.prepare();
        this.execute();

        this.eventsService.post(new ExecutablesExecutedEvent());
    }
}
