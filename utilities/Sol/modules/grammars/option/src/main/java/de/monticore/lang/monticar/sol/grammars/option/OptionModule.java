/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.common.CommonModule;
import de.monticore.lang.monticar.sol.grammars.option._cocos.OptionCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.option.cocos.NameFormatCoCo;
import de.monticore.lang.monticar.sol.grammars.option.cocos.OptionCoCo;
import de.monticore.lang.monticar.sol.grammars.option.cocos.instance.*;
import de.monticore.lang.monticar.sol.grammars.option.cocos.type.CompositeOrReturnsCoCo;
import de.monticore.lang.monticar.sol.grammars.option.cocos.type.UniqueDeclarationCoCo;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.CoCo;
import de.monticore.lang.monticar.sol.runtime.grammar.cocos.DuplicateCoCoErrorCode;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class OptionModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
        this.installModules();
    }

    private void addBindings() { /* Placeholder */ }

    private void addMultiBindings() {
        this.addContextConditions();
    }

    private void installModules() {
        this.install(new CommonModule());
    }

    private void addContextConditions() {
        Multibinder<OptionCoCo> coCos = Multibinder.newSetBinder(binder(), OptionCoCo.class);

        coCos.addBinding().to(NameFormatCoCo.class);

        coCos.addBinding().to(AssignmentTypeCoCo.class);
        coCos.addBinding().to(CompositeCoCo.class);
        coCos.addBinding().to(RequiredAssignmentCoCo.class);
        coCos.addBinding().to(SupportedAssignmentCoCo.class);
        coCos.addBinding().to(UniqueAssignmentCoCo.class);
        coCos.addBinding().to(UniqueOptionCoCo.class);
        coCos.addBinding().to(ExistingTypeCoCo.class);

        coCos.addBinding().to(UniqueDeclarationCoCo.class);
        coCos.addBinding().to(CompositeOrReturnsCoCo.class);
    }

    @Provides
    protected OptionCoCoChecker provideCoCoChecker(Set<OptionCoCo> coCos) throws DuplicateCoCoErrorCode {
        OptionCoCoChecker checker = new OptionCoCoChecker();
        Set<String> errorCodes = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toSet());
        List<String> workingList = coCos.stream().map(CoCo::getErrorCode).collect(Collectors.toList());

        errorCodes.forEach(workingList::remove);

        if (workingList.isEmpty()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new DuplicateCoCoErrorCode(workingList);

        return checker;
    }
}
