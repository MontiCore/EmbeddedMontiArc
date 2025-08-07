/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.language.cocos.*;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CoCo;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.DuplicateCoCoErrorCode;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class LanguageModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
    }

    private void addBindings() { /* Placeholder */ }

    private void addMultiBindings() {
        this.addContextConditions();
    }

    private void addContextConditions() {
        Multibinder<LanguageCoCo> coCos = Multibinder.newSetBinder(binder(), LanguageCoCo.class);

        coCos.addBinding().to(ExcludeExistingCoCo.class);
        coCos.addBinding().to(UniqueIdentifierCoCo.class);
        coCos.addBinding().to(UniqueOptionCoCo.class);
        coCos.addBinding().to(CircularDependencyCoCo.class);
    }

    @Provides
    protected LanguageCoCoChecker provideCoCoChecker(Set<LanguageCoCo> coCos, OptionCoCoChecker optionsChecker) throws DuplicateCoCoErrorCode {
        LanguageCoCoChecker checker = new LanguageCoCoChecker();
        Set<String> errorCodes = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toSet());
        List<String> workingList = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toList());

        checker.addChecker(optionsChecker);
        errorCodes.forEach(workingList::remove);

        if (workingList.isEmpty()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new DuplicateCoCoErrorCode(workingList);

        return checker;
    }
}
