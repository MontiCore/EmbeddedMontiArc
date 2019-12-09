/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;

public class ModuleConfigurationOrderCoCo extends CommonIDECoCo implements IDEASTModuleTypeCoCo {
    public ModuleConfigurationOrderCoCo() {
        super("IDE0016", "Configuration '%s' should not have an order in Module Type '%s'.");
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
        if (configuration.getOrder().isPresent()) this.warn(configuration, configuration.getName(), type.getName());
    }
}
