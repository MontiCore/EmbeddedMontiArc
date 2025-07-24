/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer.CommonPrinter;

public interface OptionPrinter extends CommonPrinter {
    String printOptionType(OptionSymbol option);
    String printOptionType(OptionSymbol option, boolean error);
}
