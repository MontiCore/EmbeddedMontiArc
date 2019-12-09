/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.WriteSymbol;

import java.util.List;
import java.util.stream.Collectors;

public class WriteModuleCoCo extends CommonIDECoCo implements IDEASTModuleTypeCoCo {
    public WriteModuleCoCo() {
        super("IDE0012", "Module '%s' has not been written to any relative path.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo(this);
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ModuleTypeSymbol symbol) {
        List<String> modules = symbol.getModuleSymbols().stream()
                .map(ModuleSymbol::getName)
                .collect(Collectors.toList());
        List<String> writes = symbol.getWriteSymbols().stream()
                .map(WriteSymbol::getName)
                .collect(Collectors.toList());

        writes.forEach(modules::remove);
        modules.forEach(module -> this.error(symbol, module));
    }
}
