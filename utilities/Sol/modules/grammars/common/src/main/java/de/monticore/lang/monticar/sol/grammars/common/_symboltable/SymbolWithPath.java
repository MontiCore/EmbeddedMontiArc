/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.Optional;

/**
 * An interface to be implemented by symbol classes which are working on path attributes.
 */
public interface SymbolWithPath extends Symbol {
    /**
     * Sets the path attribute of the symbol to a given one.
     * @param path The value to which the path attribute should be set.
     */
    void setPath(String path);

    /**
     * Returns the path attribute of the symbol wrapped in Optional.
     * @return An Optional holding the path attribute of the symbol.
     */
    Optional<String> getPath();
}
