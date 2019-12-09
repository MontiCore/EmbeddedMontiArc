/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.grammar.cocos;

/**
 * An interface to be implemented by concrete context conditions.
 * @param <CoCoChecker> The CoCoChecker which operates on the given context condition.
 */
public interface CoCo<CoCoChecker> {
    /**
     * Returns the error code of the context condition.
     * @return The error code of the context condition.
     */
    String getErrorCode();

    /**
     * Adds the context condition to a given checker.
     * @param checker The checker to which the context condition should be added.
     */
    void registerTo(CoCoChecker checker);
}
