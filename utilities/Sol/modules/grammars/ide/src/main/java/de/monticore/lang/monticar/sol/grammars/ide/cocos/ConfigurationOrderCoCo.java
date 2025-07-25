/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;

public class ConfigurationOrderCoCo extends CommonIDECoCo implements IDEASTConfigurationTypeCoCo {
    public ConfigurationOrderCoCo() {
        super("IDE0014", "Sub-Configuration '%s' needs an order inside configuration '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationTypeSymbol type) {
        type.getConfigurationSymbols().forEach(configuration -> this.check(type, configuration));
    }

    protected void check(ConfigurationTypeSymbol type, ConfigurationSymbol configuration) {
        if (!configuration.getOrder().isPresent()) this.error(configuration, configuration.getName(), type.getName());
    }
}
