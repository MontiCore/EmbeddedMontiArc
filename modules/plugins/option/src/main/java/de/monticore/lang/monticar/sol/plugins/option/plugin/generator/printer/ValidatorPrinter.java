/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator.printer;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

public interface ValidatorPrinter extends OptionPrinter {
    String printMethods(SymbolWithOptions symbol);
    String printMethods(SymbolWithOptions symbol, OptionSymbol option);
    String printBody(OptionSymbol option);
}
