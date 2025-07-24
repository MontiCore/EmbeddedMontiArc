/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.symboltable.Symbol;

import java.util.List;
import java.util.stream.Collectors;

/**
 * An interface to be implemented by symbols working on languages.
 */
public interface SymbolWithLanguages extends Symbol {
    /**
     * Adds a language to the list of languages.
     * @param language The language to be added.
     */
    void addLanguage(LanguageSymbolReference language);

    /**
     * Returns the list of languages.
     * @return The list of languages.
     */
    List<LanguageSymbolReference> getLanguages();

    /**
     * Returns the list of languages as symbols.
     * @return The list of languages as symbols.
     */
    default List<LanguageSymbol> getLanguageSymbols() {
        return this.getLanguages().stream()
                .filter(LanguageSymbolReference::existsReferencedSymbol)
                .map(LanguageSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
