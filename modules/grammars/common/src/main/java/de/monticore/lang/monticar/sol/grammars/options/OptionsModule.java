/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.options.cocos.*;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializer;
import de.monticore.lang.monticar.sol.grammars.options.visitor.OptionsSerializerImpl;

import java.util.Set;
import java.util.stream.Collectors;

public class OptionsModule extends AbstractModule {
    @Override
    protected void configure() {
        this.addBindings();
        this.addMultiBindings();
    }

    private void addBindings() {
        bind(OptionsSerializer.class).to(OptionsSerializerImpl.class);
        bind(ComponentTypeService.class).to(ComponentTypeServiceImpl.class);
    }

    private void addMultiBindings() {
        this.addContextConditions();
    }

    private void addContextConditions() {
        Multibinder<OptionCoCo> coCos = Multibinder.newSetBinder(binder(), OptionCoCo.class);

        coCos.addBinding().to(NoDuplicatePropCoCo.class);
        coCos.addBinding().to(PropTypeCoCo.class);
        coCos.addBinding().to(RequiredPropCoCo.class);
        coCos.addBinding().to(SubOptionsCoCo.class);
        coCos.addBinding().to(SupportedPropCoCo.class);
        coCos.addBinding().to(SupportedTypeCoCo.class);
        coCos.addBinding().to(UniqueOptionCoCo.class);
    }

    @Provides
    protected OptionsCoCoChecker provideCoCoChecker(Set<OptionCoCo> coCos) throws Exception {
        OptionsCoCoChecker checker = new OptionsCoCoChecker();

        Set<String> errorCodes = coCos.stream().map(OptionCoCo::getErrorCode).collect(Collectors.toSet());

        if (coCos.size() == errorCodes.size()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new Exception("The same error code has been used in more than one context condition.");

        return checker;
    }
}
