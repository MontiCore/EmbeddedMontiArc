/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.language.cocos;

import de.monticore.lang.monticar.sol.grammars.language._cocos.LanguageCoCoChecker;

/**
 * An interface implemented by all context conditions of the Language grammar.
 */
public interface LanguageCoCo {
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
     * @param checker The LanguageCoCoChecker to which the context condition will be registered.
     */
    void registerTo(LanguageCoCoChecker checker);
}
