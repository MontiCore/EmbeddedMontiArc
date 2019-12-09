/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDE;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTIDECoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;

import java.util.List;
import java.util.stream.Collectors;

public class IncludeExcludedCoCo extends CommonIDECoCo implements IDEASTIDECoCo {
    public IncludeExcludedCoCo() {
        super("IDE0005","Already included %s '%s' cannot be included again.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTIDE node) {
        node.getIDESymbolOpt().ifPresent(this::check);
    }

    protected void check(IDESymbol symbol) {
        this.checkConfigurationTypes(symbol);
        this.checkModuleTypes(symbol);
    }

    protected void checkConfigurationTypes(IDESymbol symbol) {
        List<String> effectiveNames = symbol.getInheritedConfigurationTypeSymbols().stream()
                .map(ConfigurationTypeSymbol::getFullName)
                .collect(Collectors.toList());
        List<String> inclusionNames = symbol.getConfigurationTypeInclusionSymbols().stream()
                .map(ConfigurationTypeSymbol::getFullName)
                .collect(Collectors.toList());

        inclusionNames.stream()
                .filter(effectiveNames::contains)
                .forEach(configuration -> this.error(symbol, "configuration", configuration));
    }

    protected void checkModuleTypes(IDESymbol symbol) {
        List<String> effectiveNames = symbol.getInheritedModuleTypeSymbols().stream()
                .map(ModuleTypeSymbol::getFullName)
                .collect(Collectors.toList());
        List<String> inclusionNames = symbol.getModuleTypeInclusionSymbols().stream()
                .map(ModuleTypeSymbol::getFullName)
                .collect(Collectors.toList());

        inclusionNames.stream()
                .filter(effectiveNames::contains)
                .forEach(module -> this.error(symbol, "module", module));
    }
}
