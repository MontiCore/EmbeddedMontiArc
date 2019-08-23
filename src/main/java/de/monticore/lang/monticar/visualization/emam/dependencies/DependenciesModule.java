/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.dependencies;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class DependenciesModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<DependenciesContribution> dependencies =
                Multibinder.newSetBinder(binder(), DependenciesContribution.class);

        dependencies.addBinding().to(VisualizationDependency.class);
        dependencies.addBinding().to(MathPrettyPrinterDependency.class);

        bind(DependenciesService.class).to(DependenciesServiceImpl.class);
    }
}
