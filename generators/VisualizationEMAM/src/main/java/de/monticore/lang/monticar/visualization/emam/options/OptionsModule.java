/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.options;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class OptionsModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<OptionsContribution> options =
                Multibinder.newSetBinder(binder(), OptionsContribution.class);

        options.addBinding().to(ModelOption.class);
        options.addBinding().to(ModelPathOption.class);
        options.addBinding().to(OutputPathOption.class);

        bind(OptionsService.class).to(OptionsServiceImpl.class);
    }
}
