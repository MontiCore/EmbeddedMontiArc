/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.artifact._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.Optional;

public interface SymbolWithAlias extends Symbol {
    void setAlias(String alias);
    Optional<String> getAlias();
}
