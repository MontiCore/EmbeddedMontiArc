/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.NPMPackageService;
import de.monticore.lang.monticar.sol.plugins.common.plugin.common.npm.SolPackage;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.order.OrderService;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer.ImportPrinter;

@Singleton
public class IDEGlex implements GlexContribution {
    protected final NPMPackageService packages;
    protected final IDEMethodDelegator delegator;
    protected final OrderService sorter;
    protected final ImportPrinter printer;

    @Inject
    protected IDEGlex(NPMPackageService packages, IDEMethodDelegator delegator,
                      OrderService sorter, ImportPrinter printer) {
        this.packages = packages;
        this.delegator = delegator;
        this.sorter = sorter;
        this.printer = printer;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        SolPackage solPackage = this.packages.getCurrentPackage()
                .orElseThrow(() -> new RuntimeException("Plugin is not operating on Sol package."));

        glex.defineGlobalVar("ide.delegator", this.delegator);
        glex.defineGlobalVar("ide.sorter", this.sorter);
        glex.defineGlobalVar("ide.printer", this.printer);
        glex.defineGlobalVar("solPackage", solPackage);
    }
}
