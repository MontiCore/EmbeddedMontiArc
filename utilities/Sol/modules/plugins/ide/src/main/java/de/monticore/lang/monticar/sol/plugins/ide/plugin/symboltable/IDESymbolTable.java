/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDECompilationUnit;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;

import java.util.Optional;

public interface IDESymbolTable {
    Optional<ASTIDECompilationUnit> getRootNode();
    Optional<IDESymbol> getRootSymbol();
}
