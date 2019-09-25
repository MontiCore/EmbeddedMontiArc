/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import org.eclipse.lsp4j.jsonrpc.validation.NonNull;

import java.io.File;
import java.nio.file.Path;

public interface OptionsService {
    /**
     * @param option The option for which existence should be checked.
     * @return True if the option has been used upon execution, false otherwise.
     */
    boolean hasOption(@NonNull String option);

    /**
     * @param option The option for which the value should be fetched.
     * @return The value as string passed to the command line as argument.
     */
    String getOptionAsString(@NonNull String option);

    /**
     * @param option The option for which the value should be fetched.
     * @param defaultValue The value which should be used if the command line does not include this option.
     * @return The value as string passed to the command line as argument or the default value if non-existent.
     */
    String getOptionAsString(@NonNull String option, @NonNull String defaultValue);

    /**
     * @param option The option for which the value should be fetched.
     * @return The value as {@link java.nio.file.Path Path} passed to the command line as argument.
     */
    Path getOptionAsPath(@NonNull String option);

    /**
     * @param option The option for which the value should be fetched.
     * @param defaultPath The value which should be used if the command line does not include this option.
     * @return The value as {@link java.nio.file.Path Path} passed to the command line as argument or the default value if non-existent.
     */
    Path getOptionAsPath(@NonNull String option, @NonNull String defaultPath);

    /**
     * @param option The option for which the value should be fetched.
     * @return The value as {@link java.io.File File} passed to the command line as argument.
     */
    File getOptionAsFile(@NonNull String option);

    /**
     * @param option The option for which the value should be fetched.
     * @param defaultFile The value which should be used if the command line does not include this option.
     * @return The value as {@link java.io.File File} passed to the command line as argument or the default value if non-existent.
     */
    File getOptionAsFile(@NonNull String option, @NonNull String defaultFile);

    /**
     * @param option The option for which the value should be fetched.
     * @return The value as integer passed to the command line as argument.
     */
    int getOptionAsInteger(@NonNull String option);

    /**
     * @param option The option for which the value should be fetched.
     * @param defaultValue The value which should be used if the command line does not include this option.
     * @return The value as integer passed to the command line as argument or the default value if non-existent.
     */
    int getOptionAsInteger(@NonNull String option, @NonNull String defaultValue);
}
