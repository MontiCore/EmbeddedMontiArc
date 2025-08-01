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

import java.util.Set;

public class ExcludeNeededCoCo extends CommonIDECoCo implements IDEASTIDECoCo {
    public ExcludeNeededCoCo() {
        super("IDE0015", "%s Type '%s' is needed and can therefore not be excluded.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTIDE node) {
        node.getIDESymbolOpt().ifPresent(this::check);
    }

    protected void check(IDESymbol ide) {
        this.checkConfigurations(ide);
        this.checkModules(ide);
    }

    protected void checkConfigurations(IDESymbol ide) {
        Set<ConfigurationTypeSymbol> exclusions = ide.getConfigurationTypeExclusionSymbols();
        Set<ConfigurationTypeSymbol> needed = ide.getAllNeededConfigurationTypeSymbols();

        exclusions.forEach(exclusion -> {
            if (needed.contains(exclusion)) this.error(exclusion, "Configuration", exclusion.getName());
        });
    }

    protected void checkModules(IDESymbol ide) {
        Set<ModuleTypeSymbol> exclusions = ide.getModuleTypeExclusionSymbols();
        Set<ModuleTypeSymbol> needed = ide.getAllNeededModuleTypeSymbols();

        exclusions.forEach(exclusion -> {
            if (needed.contains(exclusion)) this.error(exclusion, "Module", exclusion.getName());
        });
    }
}
