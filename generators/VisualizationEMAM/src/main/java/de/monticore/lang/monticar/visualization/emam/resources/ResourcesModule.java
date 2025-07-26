/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.resources;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class ResourcesModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<ResourcesContribution> dependencies =
                Multibinder.newSetBinder(binder(), ResourcesContribution.class);

        dependencies.addBinding().to(VisualizationResource.class);
        dependencies.addBinding().to(MathPrettyPrinterResource.class);

        bind(ResourcesService.class).to(ResourcesServiceImpl.class);
    }
}
