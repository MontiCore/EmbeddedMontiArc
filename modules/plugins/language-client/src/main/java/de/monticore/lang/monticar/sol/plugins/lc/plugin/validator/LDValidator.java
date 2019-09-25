/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.validator;

public interface LDValidator {
    /**
     * Checks whether the models on the models path are syntactically and semantically correct.
     * @throws Exception An exception which might get thrown during validation.
     */
    void validate() throws Exception;
}
