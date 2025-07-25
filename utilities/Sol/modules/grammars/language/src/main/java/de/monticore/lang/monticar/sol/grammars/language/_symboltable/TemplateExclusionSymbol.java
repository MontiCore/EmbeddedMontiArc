/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import java.util.Optional;
import java.util.function.Predicate;
import java.util.stream.Stream;

public class TemplateExclusionSymbol extends TemplateExclusionSymbolTOP {
    protected LanguageSymbolReference owner;
    protected TemplateDeclarationSymbolReference declaration;

    public TemplateExclusionSymbol(String name) {
        super(name);
    }

    public void setOwner(LanguageSymbolReference owner) {
        this.owner = owner;
    }

    public Optional<LanguageSymbolReference> getOwner() {
        return Optional.ofNullable(this.owner);
    }

    public Optional<TemplateDeclarationSymbol> getMatchingDeclarationSymbol() {
        Predicate<TemplateDeclarationSymbol> filter = symbol -> symbol.getName().equals(this.getName());

        return this.getOwner()
                .filter(LanguageSymbolReference::existsReferencedSymbol)
                .map(LanguageSymbolReference::getReferencedSymbol)
                .map(owner -> owner.getAllDeclarationSymbols().stream())
                .map(stream -> stream.filter(filter))
                .flatMap(Stream::findFirst);
    }
}
