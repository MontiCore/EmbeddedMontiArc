/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.stream.Collectors;

public interface SymbolWithFilledOptions extends Symbol {
    void addOptionFill(OptionFillSymbolReference fill);
    List<OptionFillSymbolReference> getOptionFills();

    default List<OptionFillSymbol> getOptionFillSymbols() {
        return this.getOptionFills().stream()
                .filter(OptionFillSymbolReference::existsReferencedSymbol)
                .map(OptionFillSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
