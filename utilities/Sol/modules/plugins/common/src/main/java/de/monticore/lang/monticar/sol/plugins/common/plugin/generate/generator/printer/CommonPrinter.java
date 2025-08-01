/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.common.plugin.generate.generator.printer;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValue;
import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithValueList;

public interface CommonPrinter {
    String print(SymbolWithValue symbol);
    String print(SymbolWithValueList symbol);
}
