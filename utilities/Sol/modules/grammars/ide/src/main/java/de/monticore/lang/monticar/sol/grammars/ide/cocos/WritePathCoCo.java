/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDEASTModuleTypeCoCo;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.IDECoCoChecker;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.WriteSymbol;

import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

public class WritePathCoCo extends CommonIDECoCo implements IDEASTModuleTypeCoCo {
    public WritePathCoCo() {
        super("IDE0011", "More than one write operation is pointing to the same relative path '%s'.");
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
        List<String> relativePaths = symbol.getWriteSymbols().stream()
                .map(WriteSymbol::getRelativePath)
                .collect(Collectors.toList());
        Set<String> uniqueRelativePaths = new HashSet<>(relativePaths);

        uniqueRelativePaths.forEach(relativePaths::remove);
        relativePaths.forEach(relativePath -> this.error(symbol, relativePath));
    }
}
