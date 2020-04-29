/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.lc.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.grammars.language._symboltable.LanguageSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.configuration.LanguageClientConfiguration;
import de.monticore.lang.monticar.sol.plugins.lc.plugin.symboltable.LanguageSymbolTable;

@Singleton
public class LanguageClientGlex implements GlexContribution {
    protected final LanguageClientConfiguration configuration;
    protected final NPMPackageService packages;
    protected final LanguageSymbolTable symbolTable;

    @Inject
    protected LanguageClientGlex(LanguageClientConfiguration configuration, NPMPackageService packages,
                                 LanguageSymbolTable symbolTable) {
        this.configuration = configuration;
        this.packages = packages;
        this.symbolTable = symbolTable;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        SolPackage rootPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new RuntimeException("Root Package should be defined by this point."));
        LanguageSymbol rootSymbol = this.symbolTable.getRootSymbol()
                .orElseThrow(() -> new RuntimeException("Root Symbol could not be resolved."));

        glex.defineGlobalVar("configuration", this.configuration);
        glex.defineGlobalVar("rootPackage", rootPackage);
        glex.defineGlobalVar("rootSymbol", rootSymbol);
    }
}
