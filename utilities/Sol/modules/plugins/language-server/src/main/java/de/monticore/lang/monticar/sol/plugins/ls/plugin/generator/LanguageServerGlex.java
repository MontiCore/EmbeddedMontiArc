/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ls.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.ls.plugin.configuration.LanguageServerConfiguration;

@Singleton
public class LanguageServerGlex implements GlexContribution {
    protected final LanguageServerConfiguration configuration;

    @Inject
    protected LanguageServerGlex(LanguageServerConfiguration configuration) {
        this.configuration = configuration;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        glex.defineGlobalVar("configuration", this.configuration);
    }
}
