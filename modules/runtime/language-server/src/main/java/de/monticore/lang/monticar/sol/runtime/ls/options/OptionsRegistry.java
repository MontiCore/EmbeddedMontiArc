/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.options;

import org.apache.commons.cli.Option;
import org.apache.commons.cli.Options;

public interface OptionsRegistry {
    /**
     * @return The options registered in the registry.
     */
    Options getOptions();

    /**
     * A method which will be used to add an option to the registry.
     * @param option The option to be added.
     */
    void registerOption(Option option);
}
