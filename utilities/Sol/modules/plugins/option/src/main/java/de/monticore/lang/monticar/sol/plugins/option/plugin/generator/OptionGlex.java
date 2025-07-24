/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.templateengine.GlobalExtensionManagement;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.GlexContribution;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer.ValidatorPrinter;
import de.monticore.lang.monticar.sol.plugins.option.plugin.generator.serializer.OptionsSerializer;

@Singleton
public class OptionGlex implements GlexContribution {
    protected final OptionMethodDelegator delegator;
    protected final OptionsSerializer serializer;
    protected final ValidatorPrinter printer;

    @Inject
    protected OptionGlex(OptionMethodDelegator delegator, OptionsSerializer serializer, ValidatorPrinter printer) {
        this.delegator = delegator;
        this.serializer = serializer;
        this.printer = printer;
    }

    @Override
    public void defineGlobalVars(GlobalExtensionManagement glex) {
        glex.defineGlobalVar("option.delegator", this.delegator);
        glex.defineGlobalVar("option.serializer", this.serializer);
        glex.defineGlobalVar("option.printer", this.printer);
    }
}
