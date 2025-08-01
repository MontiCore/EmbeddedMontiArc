/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionInheritSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;

import java.util.Map;

public class OptionInheritTypeCoCo extends OptionCommonCoCo implements IDEASTConfigurationTypeCoCo {
    public OptionInheritTypeCoCo() {
        super("IDE0008", "Option '%s' can only inherit from an option of the same type '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationTypeSymbol symbol) {
        Map<String, String> inheritedTypeMap = this.getTypeMapOf(symbol);

        symbol.getConfigurationSymbols().forEach(configuration -> this.check(configuration, inheritedTypeMap));
    }

    protected void check(ConfigurationSymbol symbol, Map<String, String> inheritedTypeMap) {
        Map<String, String> typeMap = this.getTypeMapOf(symbol);

        symbol.getOptionInheritSymbols().forEach(inherit -> this.check(inherit, inheritedTypeMap, typeMap));
    }

    protected void check(OptionInheritSymbol inherit, Map<String, String> inheritedTypeMap, Map<String, String> typeMap) {
        String optionName = inherit.getOptionSymbol()
                .map(OptionSymbol::getName)
                .orElse("UNKNOWN_OPTION");
        String parentName = inherit.getParentSymbol()
                .map(OptionSymbol::getName)
                .orElse("UNKNOWN_PARENT");
        String optionType = typeMap.getOrDefault(optionName, "UNKNOWN_TYPE");
        String parentType = inheritedTypeMap.getOrDefault(parentName, "UNKNOWN_PARENT_TYPE");

        if (!optionType.equals(parentType)) this.error(inherit, optionName, optionType);
    }
}
