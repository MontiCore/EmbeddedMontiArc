/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import java.util.List;
import java.util.stream.Collectors;

public interface SymbolWithInheritedOptions extends SymbolWithFilledOptions {
    void addOptionInherit(OptionInheritSymbolReference inherit);
    List<OptionInheritSymbolReference> getOptionInherits();

    default List<OptionInheritSymbol> getOptionInheritSymbols() {
        return this.getOptionInherits().stream()
                .filter(OptionInheritSymbolReference::existsReferencedSymbol)
                .map(OptionInheritSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
