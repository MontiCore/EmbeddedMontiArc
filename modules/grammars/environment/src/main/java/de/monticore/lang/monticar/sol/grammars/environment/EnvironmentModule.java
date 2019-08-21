/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.environment;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.environment._cocos.EnvironmentCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.environment.cocos.*;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Set;
import java.util.stream.Collectors;

public class EnvironmentModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addMultiBindings();
    }

    private void addMultiBindings() {
        this.addContextConditions();
    }

    private void addContextConditions() {
        Multibinder<EnvironmentCoCo> coCos = Multibinder.newSetBinder(binder(), EnvironmentCoCo.class);

        coCos.addBinding().to(NoAPTGetInstallCoCo.class);
        coCos.addBinding().to(NoCommandChainingCoCo.class);
        coCos.addBinding().to(ValidateEnvNameCoCo.class);
        coCos.addBinding().to(ValidatePortCoCo.class);
        coCos.addBinding().to(NoRelativeWorkDirCoCo.class);
        coCos.addBinding().to(BlockedInstructionCoCo.class);
        coCos.addBinding().to(ImportComponentCoCo.class);
    }

    @Provides
    protected EnvironmentCoCoChecker provideCoCoChecker(Set<EnvironmentCoCo> coCos) throws Exception {
        EnvironmentCoCoChecker checker = new EnvironmentCoCoChecker();

        Set<String> errorCodes = coCos.stream().map(EnvironmentCoCo::getErrorCode).collect(Collectors.toSet());

        if (coCos.size() == errorCodes.size()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new Exception("The same error code has been used in more than one context condition.");

        return checker;
    }

    @Provides
    protected ResolvingConfiguration providesResolvingConfiguration(EnvironmentLanguage language) {
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        configuration.addDefaultFilters(language.getResolvingFilters());

        return configuration;
    }
}
