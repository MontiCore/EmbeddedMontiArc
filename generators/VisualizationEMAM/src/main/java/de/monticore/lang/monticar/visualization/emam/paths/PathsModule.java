/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.paths;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class PathsModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<PathsContribution> paths =
                Multibinder.newSetBinder(binder(), PathsContribution.class);

        paths.addBinding().to(VisualizationPath.class);
        paths.addBinding().to(MathPrettyPrinterPath.class);

        bind(PathsService.class).to(PathsServiceImpl.class);
    }
}
