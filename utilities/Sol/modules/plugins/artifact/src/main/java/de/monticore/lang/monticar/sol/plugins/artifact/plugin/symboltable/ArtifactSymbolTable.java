/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.artifact.plugin.symboltable;

import de.monticore.lang.monticar.sol.grammars.artifact._ast.ASTArtifactCompilationUnit;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ArtifactSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;

import java.util.List;

public interface ArtifactSymbolTable {
    List<ASTArtifactCompilationUnit> getRootNodes();
    List<ArtifactSymbol> getArtifactSymbols();
    List<ToolSymbol> getAllToolSymbols();
    List<ToolSymbol> getToolSymbols();
    List<ToolSymbol> getVirtualToolSymbols();
}
