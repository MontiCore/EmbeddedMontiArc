/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.stream.Collectors;

/**
 * An interface to be implemented by symbols which are operating on options.
 */
public interface SymbolWithOptions extends Symbol {
    /**
     * Adds an option to the collection of options.
     * @param option The option to be added.
     */
    void addOption(OptionSymbolReference option);

    /**
     * Returns the options present in the collection.
     * @return A list of options present in the collection.
     */
    List<OptionSymbolReference> getOptions();

    /**
     * Returns the options present in the collection as symbols.
     * @return A list of OptionSymbols derived from the options present in the collection.
     */
    default List<OptionSymbol> getOptionSymbols() {
        return this.getOptions().stream()
                .filter(OptionSymbolReference::existsReferencedSymbol)
                .map(OptionSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
