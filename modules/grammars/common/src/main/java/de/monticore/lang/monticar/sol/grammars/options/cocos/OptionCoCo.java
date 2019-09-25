/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options.cocos;

import de.monticore.lang.monticar.sol.grammars.options._cocos.OptionsCoCoChecker;

/**
 * An interface implemented by all context conditions of the Options grammar.
 */
public interface OptionCoCo {
    /**
     * @return The error code used by this context condition.
     */
    String getErrorCode();

    /**
     * @param parameters Additional parameters to be used in the construction of the error message.
     * @return The message to be added to the Log.
     */
    String getErrorMessage(Object ...parameters);

    /**
     * This method will be called in order to register a context condition to a checker.
     * @param checker The OptionsCoCoChecker to which the context condition will be registered.
     */
    void registerTo(OptionsCoCoChecker checker);
}
