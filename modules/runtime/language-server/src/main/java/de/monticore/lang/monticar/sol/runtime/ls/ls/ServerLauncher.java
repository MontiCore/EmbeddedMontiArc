/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import org.eclipse.lsp4j.jsonrpc.validation.NonNull;

public interface ServerLauncher {
    /**
     * Initializes the application.
     * @param arguments The arguments passed to the application upon execution.
     * @throws Exception An exception which might occur during initialization.
     */
    void initialize(@NonNull String[] arguments) throws Exception;

    /**
     * Configures the application.
     * @param arguments The arguments passed to the application upon execution.
     * @throws Exception An exception which might occur during configuration.
     */
    void configure(@NonNull String[] arguments) throws Exception;

    /**
     * Launches the application.
     * @param arguments The arguments passed to the application upon execution.
     * @throws Exception An exception which might occur during launch.
     */
    void launch(@NonNull String[] arguments) throws Exception;
}
