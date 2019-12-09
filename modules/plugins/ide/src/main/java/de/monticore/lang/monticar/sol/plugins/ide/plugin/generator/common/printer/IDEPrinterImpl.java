/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import com.google.inject.Inject;
import com.google.inject.Singleton;
import de.monticore.generating.GeneratorEngine;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralFillSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralListFillSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer.CommonPrinterImpl;

@Singleton
public class IDEPrinterImpl extends CommonPrinterImpl implements IDEPrinter {
    protected static final String DEFAULT = "undefined";

    @Inject
    protected IDEPrinterImpl(GeneratorEngine engine) {
        super(engine, "templates/ide");
    }

    @Override
    public String printOptionFillValue(OptionFillSymbol fill) {
        if (fill.isLiteralFill()) return fill.asLiteralFill().map(this::printLiteralFillValue).orElse(DEFAULT);
        else if (fill.isLiteralListFill()) return fill.asLiteralListFill().map(this::printLiteralListFillValue).orElse(DEFAULT);
        return DEFAULT;
    }

    protected String printLiteralFillValue(LiteralFillSymbol fill) {
        return super.print(fill);
    }

    protected String printLiteralListFillValue(LiteralListFillSymbol fill) {
        return super.print(fill);
    }
}
