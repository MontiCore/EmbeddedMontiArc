/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.visualization.emam.executables;

import com.google.inject.AbstractModule;
import com.google.inject.multibindings.Multibinder;

public class ExecutablesModule extends AbstractModule {
    @Override
    public void configure() {
        Multibinder<ExecutablesContribution> executables =
                Multibinder.newSetBinder(binder(), ExecutablesContribution.class);

        executables.addBinding().to(VisualizationExecutable.class);
        executables.addBinding().to(MathPrettyPrinterExecutable.class);

        bind(ExecutablesService.class).to(ExecutablesServiceImpl.class);
    }
}
