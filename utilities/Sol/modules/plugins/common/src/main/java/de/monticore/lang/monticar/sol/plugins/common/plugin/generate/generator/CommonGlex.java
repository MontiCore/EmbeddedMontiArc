/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer.CommonPrinter;

@Singleton
public class CommonGlex implements GlexContribution {
    protected final CommonMethodDelegator delegator;
    protected final CommonPrinter printer;

    @Inject
    protected CommonGlex(CommonMethodDelegator delegator, CommonPrinter printer) {
        this.delegator = delegator;
        this.printer = printer;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        glex.defineGlobalVar("common.delegator", this.delegator);
        glex.defineGlobalVar("common.printer", this.printer);
    }
}
