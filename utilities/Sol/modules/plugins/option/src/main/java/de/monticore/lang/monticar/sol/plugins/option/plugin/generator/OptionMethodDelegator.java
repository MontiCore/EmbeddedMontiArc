/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

import java.util.List;

/**
 * ATTENTION READER: This interface and its corresponding implementation are necessary because the Freemarker
 * version of MontiCore does not support the default methods of Java 8.
 */
public interface OptionMethodDelegator {
    List<OptionSymbol> getOptionSymbols(SymbolWithOptions symbol);
}
