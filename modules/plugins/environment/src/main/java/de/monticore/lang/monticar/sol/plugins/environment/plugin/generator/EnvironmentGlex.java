/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.environment.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.environment.plugin.configuration.EnvironmentGenerateConfiguration;

@Singleton
public class EnvironmentGlex implements GlexContribution {
    protected final EnvironmentGenerateConfiguration configuration;

    @Inject
    protected EnvironmentGlex(EnvironmentGenerateConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        glex.defineGlobalVar("configuration", this.configuration);
    }
}
