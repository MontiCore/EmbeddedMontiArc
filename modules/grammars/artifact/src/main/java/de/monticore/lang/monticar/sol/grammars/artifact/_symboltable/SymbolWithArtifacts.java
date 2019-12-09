/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.stream.Collectors;

public interface SymbolWithArtifacts extends Symbol {
    void addArtifact(ArtifactSymbolReference resource);
    List<ArtifactSymbolReference> getArtifacts();

    default List<ArtifactSymbol> getArtifactSymbols() {
        return this.getArtifacts().stream()
                .filter(ArtifactSymbolReference::existsReferencedSymbol)
                .map(ArtifactSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
