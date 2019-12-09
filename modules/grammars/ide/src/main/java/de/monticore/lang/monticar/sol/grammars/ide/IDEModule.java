/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide.cocos.*;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CoCo;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.DuplicateCoCoErrorCode;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class IDEModule extends AbstractModule {
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
        Multibinder<IDECoCo> coCos = Multibinder.newSetBinder(binder(), IDECoCo.class);

        coCos.addBinding().to(AtLeastOneTaskCoCo.class);
        coCos.addBinding().to(CircularDependencyCoCo.class);
        coCos.addBinding().to(ComponentInclusionCoCo.class);
        coCos.addBinding().to(ConfigurationNameCoCo.class);
        coCos.addBinding().to(DuplicateIdentifierCoCo.class);
        coCos.addBinding().to(ExcludeIncludedCoCo.class);
        coCos.addBinding().to(IncludeExcludedCoCo.class);
        coCos.addBinding().to(NameFormatCoCo.class);
        coCos.addBinding().to(OptionFillTypeCoCo.class);
        coCos.addBinding().to(OptionInheritTypeCoCo.class);
        coCos.addBinding().to(OptionsCoCo.class);
        coCos.addBinding().to(RequiredAttributesCoCo.class);
        coCos.addBinding().to(WriteModuleCoCo.class);
        coCos.addBinding().to(WritePathCoCo.class);
        coCos.addBinding().to(ConfigurationOrderCoCo.class);
        coCos.addBinding().to(ExcludeNeededCoCo.class);
        coCos.addBinding().to(ModuleConfigurationOrderCoCo.class);
        coCos.addBinding().to(ExistingReferenceCoCo.class);
        coCos.addBinding().to(NoComponentInModuleCoCo.class);
    }

    @Provides
    protected IDECoCoChecker provideCoCoChecker(Set<IDECoCo> coCos, OptionCoCoChecker optionsChecker) throws DuplicateCoCoErrorCode {
        IDECoCoChecker checker = new IDECoCoChecker();
        Set<String> errorCodes = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toSet());
        List<String> workingList = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toList());

        checker.addChecker(optionsChecker);
        errorCodes.forEach(workingList::remove);

        if (workingList.isEmpty()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new DuplicateCoCoErrorCode(workingList);

        return checker;
    }
}
