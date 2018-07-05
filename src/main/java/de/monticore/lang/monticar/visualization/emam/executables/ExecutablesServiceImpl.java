package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;
import de.monticore.lang.monticar.visualization.emam.events.EventsService;

import java.util.logging.Logger;

@Singleton
public class ExecutablesServiceImpl implements ExecutablesService, ApplicationContribution {
    protected final EventsService eventsService;

    @Inject
    public ExecutablesServiceImpl(Logger logger, EventsService eventsService) {
        this.eventsService = eventsService;
    }

    @Override
    public void prepare(Application application) {
        this.eventsService.register(this);
    }
}
