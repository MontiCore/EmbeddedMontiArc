/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.language;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options.OptionsModule;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;

public class LanguageModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() {

    }

    private void addMultiBindings() {

    }

    private void installModules() {
        this.install(new OptionsModule());
    }

    @Provides
    protected LanguageCoCoChecker provideCoCoChecker(OptionsCoCoChecker optionsChecker) {
        LanguageCoCoChecker checker = new LanguageCoCoChecker();

        checker.addChecker(optionsChecker);

        return checker;
    }
}
