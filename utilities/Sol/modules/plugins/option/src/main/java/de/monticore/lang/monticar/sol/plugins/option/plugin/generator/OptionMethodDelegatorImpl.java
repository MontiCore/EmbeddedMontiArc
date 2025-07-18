/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.option.plugin.generator;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

import java.util.List;

public class OptionMethodDelegatorImpl implements OptionMethodDelegator {
    @Override
    public List<OptionSymbol> getOptionSymbols(SymbolWithOptions symbol) {
        return symbol.getOptionSymbols();
    }
}
