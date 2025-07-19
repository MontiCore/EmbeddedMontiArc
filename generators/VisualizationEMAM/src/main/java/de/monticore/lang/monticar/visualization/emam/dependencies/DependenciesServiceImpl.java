/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.lang.monticar.visualization.emam.application.Application;
import de.monticore.lang.monticar.visualization.emam.application.ApplicationContribution;

import java.util.Set;
import java.util.logging.Logger;

@Singleton
public class DependenciesServiceImpl implements DependenciesService, ApplicationContribution {
    protected final Logger logger;
    protected final Set<DependenciesContribution> contributions;

    @Inject
    public DependenciesServiceImpl(Logger logger, Set<DependenciesContribution> contributions) {
        this.logger = logger;
        this.contributions = contributions;
    }

    @Override
    public void prepare(Application application) throws Exception {
        this.install();
    }

    @Override
    public void install() throws Exception {
        this.logger.info("Installing Dependencies...");

        for (DependenciesContribution contribution : this.contributions) {
            if (!contribution.isInstalled()) contribution.install();
        }
    }
}
