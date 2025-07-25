/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.application;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.visualization.emam.dependencies.DependenciesServiceImpl;
import de.monticore.lang.monticar.visualization.emam.executables.ExecutablesServiceImpl;
import de.monticore.lang.monticar.visualization.emam.generator.HTMLGeneratorImpl;
import de.monticore.lang.monticar.visualization.emam.models.ModelsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.options.OptionsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.paths.PathsServiceImpl;
import de.monticore.lang.monticar.visualization.emam.resources.ResourcesServiceImpl;

public class ApplicationModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<ApplicationContribution> contributions =
                Multibinder.newSetBinder(binder(), ApplicationContribution.class);

        contributions.addBinding().to(DependenciesServiceImpl.class);
        contributions.addBinding().to(ExecutablesServiceImpl.class);
        contributions.addBinding().to(OptionsServiceImpl.class);
        contributions.addBinding().to(ResourcesServiceImpl.class);
        contributions.addBinding().to(PathsServiceImpl.class);
        contributions.addBinding().to(ModelsServiceImpl.class);
        contributions.addBinding().to(HTMLGeneratorImpl.class);
    }
}
