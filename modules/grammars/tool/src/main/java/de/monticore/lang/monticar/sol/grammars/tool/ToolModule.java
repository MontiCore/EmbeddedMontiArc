/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool;

import com.google.inject.AbstractModule;
import com.google.inject.Provides;
import com.google.inject.multibindings.Multibinder;
import de.monticore.ModelingLanguageFamily;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.EnvironmentLanguage;
import de.monticore.lang.monticar.sol.grammars.tool._cocos.ToolCoCoChecker;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.ToolLanguage;
import de.monticore.lang.monticar.sol.grammars.tool.cocos.*;
import de.monticore.symboltable.ResolvingConfiguration;

import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class ToolModule extends AbstractModule {
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

    private void installModules() { /* Placeholder */ }

    private void addContextConditions() {
        Multibinder<ToolCoCo> coCos = Multibinder.newSetBinder(binder(), ToolCoCo.class);

        coCos.addBinding().to(AttributeTypeCoCo.class);
        coCos.addBinding().to(RequiredAttributesCoCo.class);
        coCos.addBinding().to(ResourceNameCoCo.class);
        coCos.addBinding().to(RootNameCoCo.class);
        coCos.addBinding().to(UniqueAttributeCoCo.class);
        coCos.addBinding().to(UniqueResourceCoCo.class);
        coCos.addBinding().to(ValidAttributeCoCo.class);
    }

    @Provides
    protected ToolCoCoChecker provideCoCoChecker(Set<ToolCoCo> coCos) throws Exception {
        ToolCoCoChecker checker = new ToolCoCoChecker();
        Set<String> errorCodes = coCos.stream().map(ToolCoCo::getErrorCode).collect(Collectors.toSet());
        List<String> workingList = coCos.stream().map(ToolCoCo::getErrorCode).collect(Collectors.toList());

        errorCodes.forEach(workingList::remove);

        if (workingList.isEmpty()) coCos.forEach(coCo -> coCo.registerTo(checker));
        else throw new Exception(String.format("%s have already been used as error codes.", workingList));

        return checker;
    }

    @Provides
    protected ResolvingConfiguration providesResolvingConfiguration(ModelingLanguageFamily family) {
        ResolvingConfiguration configuration = new ResolvingConfiguration();

        configuration.addDefaultFilters(family.getAllResolvers());

        return configuration;
    }

    @Provides
    protected ModelingLanguageFamily providesModelingLanguageFamily(ToolLanguage tool, EnvironmentLanguage environment) {
        ModelingLanguageFamily family = new ModelingLanguageFamily();

        family.addModelingLanguage(tool);
        family.addModelingLanguage(environment);

        return family;
    }
}
