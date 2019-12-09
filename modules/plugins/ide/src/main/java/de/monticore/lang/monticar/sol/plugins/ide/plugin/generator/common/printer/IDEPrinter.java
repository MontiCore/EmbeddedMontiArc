/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.printer;

import de.monticore.lang.monticar.sol.grammars.ide._symboltable.OptionFillSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer.CommonPrinter;

public interface IDEPrinter extends CommonPrinter {
    String printOptionFillValue(OptionFillSymbol fill);
}
