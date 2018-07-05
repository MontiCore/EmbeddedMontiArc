package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.common.eventbus.EventBus;
import com.google.inject.Inject;
import de.monticore.lang.monticar.visualization.emam.paths.PathsService;
import de.monticore.lang.monticar.visualization.emam.url.URLService;

import java.util.logging.Logger;

public abstract class AbstractDependenciesContribution extends EventBus implements DependenciesContribution {
    protected final Logger logger;
    protected final PathsService pathsService;
    protected final URLService urlService;

    @Inject
    public AbstractDependenciesContribution(Logger logger, PathsService pathsService, URLService urlService) {
        this.logger = logger;
        this.pathsService = pathsService;
        this.urlService = urlService;
    }
}
