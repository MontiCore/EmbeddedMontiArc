/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.Optional;

/**
 * An interface to be implemented by symbols which are working on a type attribute.
 */
public interface SymbolWithType extends Symbol {
    /**
     * Sets the type attribute of the symbol to the specified value.
     * @param type The value to which the path attribute should be set to.
     */
    default void setType(String type) {}

    /**
     * Returns the type attribute of the symbol wrapped in Optional.
     * @return The path attribute of the symbol wrapped in Optional.
     */
    Optional<String> getType();
}
