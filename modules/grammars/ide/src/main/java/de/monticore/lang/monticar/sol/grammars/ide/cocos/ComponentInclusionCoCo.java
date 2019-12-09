/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDE;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTIDECoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;

public class ComponentInclusionCoCo extends CommonIDECoCo implements IDEASTIDECoCo {
    public ComponentInclusionCoCo() {
        super("IDE0010", "A component configuration type such as '%s' cannot be included.");
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
        ide.getEffectiveConfigurationTypeSymbols().forEach(configuration -> {
            if (configuration.isComponent()) this.error(ide, configuration.getName());
        });
    }
}
