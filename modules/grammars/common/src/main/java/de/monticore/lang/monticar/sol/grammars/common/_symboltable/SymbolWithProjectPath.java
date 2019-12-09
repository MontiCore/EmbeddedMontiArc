/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.common._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;

import java.util.Optional;

/**
 * An interface to be implemented by symbols which are working on paths resolved from a certain project origin.
 */
public interface SymbolWithProjectPath extends SymbolWithPath {
    /**
     * Sets the origin from which the path should be resolved from.
     * @param origin The value to which the origin should be set to.
     */
    void setOrigin(CommonLiterals origin);

    /**
     * Returns the origin set in the symbol wrapped in Optional.
     * @return The value of the origin set in the symbol as Optional.
     */
    Optional<CommonLiterals> getOrigin();
}
