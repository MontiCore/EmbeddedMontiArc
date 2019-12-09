/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;

public class NoComponentInModuleCoCo extends CommonIDECoCo implements IDEASTModuleTypeCoCo {
    public NoComponentInModuleCoCo() {
        super("IDE0018", "Configuration '%s' in Module Type '%s' cannot be a component.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ModuleTypeSymbol type) {
        type.getConfigurationSymbols().forEach(configuration -> this.check(type, configuration));
    }

    protected void check(ModuleTypeSymbol type, ConfigurationSymbol configuration) {
        String configurationName = configuration.getName();
        String typeName = type.getName();

        configuration.getTypeSymbol()
                .filter(ConfigurationTypeSymbol::isComponent)
                .ifPresent(t -> this.error(configuration, configurationName, typeName));
    }
}
