/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDE;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTConfigurationTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTIDECoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.SymbolWithTypeAttribute;

public class RequiredAttributesCoCo extends CommonIDECoCo
        implements IDEASTIDECoCo, IDEASTModuleTypeCoCo, IDEASTConfigurationTypeCoCo {
    public RequiredAttributesCoCo() {
        super("IDE0002", "'%s' is required in '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo((IDEASTIDECoCo) this);
        checker.addCoCo((IDEASTModuleTypeCoCo) this);
        checker.addCoCo((IDEASTConfigurationTypeCoCo) this);
    }

    @Override
    public void check(ASTIDE node) {
        node.getIDESymbolOpt().ifPresent(this::check);
    }

    protected void check(IDESymbol symbol) {
        String name = symbol.getName();

        if (!symbol.getRegistry().isPresent()) this.error(symbol, "registry", name);
        if (!symbol.getBuildPath().isPresent()) this.error(symbol, "build", name);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(SymbolWithTypeAttribute symbol) {
        String name = symbol.getName();

        if (symbol.getLabel().equals("Unknown Label")) this.error(symbol, "label", name);
    }
}
