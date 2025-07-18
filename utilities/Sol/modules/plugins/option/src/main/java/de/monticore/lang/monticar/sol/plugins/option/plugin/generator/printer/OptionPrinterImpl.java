/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionTypeSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer.CommonPrinterImpl;

@Singleton
public class OptionPrinterImpl extends CommonPrinterImpl implements OptionPrinter {
    @Inject
    protected OptionPrinterImpl(GeneratorEngine engine) {
        super(engine, "templates/option/common");
    }

    @Override
    public String printOptionType(OptionSymbol option) {
        return this.printOptionType(option, false);
    }

    @Override
    public String printOptionType(OptionSymbol option, boolean error) {
        return option.getTypeSymbol().map(type -> this.doPrintOptionType(option, type, error)).orElse(ERROR);
    }

    protected String doPrintOptionType(OptionSymbol option, OptionTypeSymbol type, boolean error) {
        if (type.isArray()) return this.generate("array", option, error);
        else if (type.isObject()) return this.generate("object", option, error);
        else return this.generate("returns", type.getReturnType().orElse("any"), error);
    }
}
