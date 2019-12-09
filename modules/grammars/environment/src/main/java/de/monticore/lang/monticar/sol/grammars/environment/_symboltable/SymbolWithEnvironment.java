/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public interface SymbolWithEnvironment extends Symbol {
    void setEnvironment(DockerfileSymbolReference environment);
    Optional<DockerfileSymbolReference> getEnvironment();

    default Optional<DockerfileSymbol> getEnvironmentSymbol() {
        return this.getEnvironment()
                .filter(DockerfileSymbolReference::existsReferencedSymbol)
                .map(DockerfileSymbolReference::getReferencedSymbol);
    }

    default Set<Integer> getAllPorts() {
        return this.getEnvironmentSymbol()
                .map(DockerfileSymbol::getAllPorts)
                .orElse(new HashSet<>());
    }
}
