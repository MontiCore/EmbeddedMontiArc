/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import org.eclipse.lsp4j.jsonrpc.validation.NonNull;

public interface OptionsContribution {
    /**
     * A method which can be used to add options to the options registry.
     * @param registry The registry to which the options will be added.
     */
    void registerOptions(@NonNull OptionsRegistry registry);
}
