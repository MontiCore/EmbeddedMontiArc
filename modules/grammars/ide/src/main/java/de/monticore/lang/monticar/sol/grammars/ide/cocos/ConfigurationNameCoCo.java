/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;

public class ConfigurationNameCoCo extends CommonIDECoCo implements IDEASTModuleTypeCoCo {
    public ConfigurationNameCoCo() {
        super("IDE0013", "Configuration '%s' in module '%s' needs an attribute 'name'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ModuleTypeSymbol module) {
        module.getConfigurationSymbols().forEach(configuration -> this.check(module, configuration));
    }

    protected void check(ModuleTypeSymbol module, ConfigurationSymbol configuration) {
        if (!configuration.getDisplayName().isPresent()) this.error(configuration, configuration.getName(), module.getName());
    }
}
