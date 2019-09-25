/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.runtime.ls.ls;

import com.google.inject.Singleton;
import de.monticore.lang.monticar.sol.runtime.ls.options.OptionsContribution;
import de.monticore.lang.monticar.sol.runtime.ls.options.OptionsRegistry;
import org.apache.commons.cli.Option;

@Singleton
public class PortOption implements OptionsContribution {
    public static final String OPTION = "port";
    public static final String DESCRIPTION = "If set, defines on which port the server should be launched.";

    @Override
    public void registerOptions(OptionsRegistry registry) {
        registry.registerOption(new Option(OPTION, true, DESCRIPTION));
    }
}
