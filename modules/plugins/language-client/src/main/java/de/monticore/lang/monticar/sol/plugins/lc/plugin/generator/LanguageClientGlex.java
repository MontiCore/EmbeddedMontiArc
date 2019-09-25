/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;

@Singleton
public class LanguageClientGlex implements GlexContribution {
    protected final LanguageClientConfiguration configuration;
    protected final NPMPackageService packages;

    @Inject
    protected LanguageClientGlex(LanguageClientConfiguration configuration, NPMPackageService packages) {
        this.configuration = configuration;
        this.packages = packages;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        SolPackage rootPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new RuntimeException("Root Package should be defined by this point."));

        glex.defineGlobalVar("configuration", this.configuration);
        glex.defineGlobalVar("rootPackage", rootPackage);
    }
}
