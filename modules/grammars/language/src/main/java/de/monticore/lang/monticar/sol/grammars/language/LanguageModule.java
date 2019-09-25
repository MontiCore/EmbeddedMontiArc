/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageLanguage;
import de.monticore.lang.monticar.sol.grammars.language.cocos.LanguageCoCo;
import de.monticore.lang.monticar.sol.grammars.language.cocos.UndeclareExistingCoCo;
import de.monticore.lang.monticar.sol.grammars.language.cocos.UniqueIdentifierCoCo;
import de.monticore.lang.monticar.sol.grammars.language.cocos.UniqueOptionCoCo;
import de.monticore.lang.monticar.sol.grammars.options.OptionsModule;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.Set;
import java.util.stream.Collectors;

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
        this.addContextConditions();
    }

    private void installModules() {
        this.install(new OptionsModule());
    }

    private void addContextConditions() {
        Multibinder<LanguageCoCo> coCos = Multibinder.newSetBinder(binder(), LanguageCoCo.class);

        coCos.addBinding().to(UndeclareExistingCoCo.class);
        coCos.addBinding().to(UniqueIdentifierCoCo.class);
        coCos.addBinding().to(UniqueOptionCoCo.class);
    }

    @Provides
    protected LanguageCoCoChecker provideCoCoChecker(Set<LanguageCoCo> coCos, OptionsCoCoChecker optionsChecker) throws Exception {
        LanguageCoCoChecker checker = new LanguageCoCoChecker();
        Set<String> errorCodes = coCos.stream().map(LanguageCoCo::getErrorCode).collect(Collectors.toSet());

        checker.addChecker(optionsChecker);

        if (coCos.size() == errorCodes.size()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new Exception("The same error code has been used in more than one context condition.");

        return checker;
    }

    @Provides
    protected ResolvingConfiguration providesResolvingConfiguration(LanguageLanguage language) {
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        configuration.addDefaultFilters(language.getResolvingFilters());

        return configuration;
    }
}
