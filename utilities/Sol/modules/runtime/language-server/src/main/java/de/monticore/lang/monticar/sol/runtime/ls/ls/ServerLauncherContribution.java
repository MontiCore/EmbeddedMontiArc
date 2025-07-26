/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.eclipse.lsp4j.jsonrpc.validation.NonNull;

public interface ServerLauncherContribution {
    /**
     * A method which is executed when the application is being initialized.
     * @param arguments The arguments passed to the application upon execution.
     * @throws Exception An exception which might occur during initialization.
     */
    default void onInitialize(@NonNull String[] arguments) throws Exception {}

    /**
     * A method which is executed when the application is being configured.
     * @param arguments The arguments passed to the application upon execution.
     * @throws Exception An exception which might occur during initialization.
     */
    default void onConfigure(@NonNull String[] arguments) throws Exception {}
}
