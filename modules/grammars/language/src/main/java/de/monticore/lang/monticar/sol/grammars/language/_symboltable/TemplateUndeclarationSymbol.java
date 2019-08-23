/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import java.util.Optional;

public class TemplateUndeclarationSymbol extends TemplateUndeclarationSymbolTOP {
    protected LanguageSymbol owner;
    protected TemplateDeclarationSymbolReference declaration;

    public TemplateUndeclarationSymbol(String name) {
        super(name);
    }

    public void setLanguageSymbol(LanguageSymbol owner) {
        this.owner = owner;
    }

    public LanguageSymbol getLanguageSymbol() {
        return this.owner;
    }

    public Optional<TemplateDeclarationSymbol> getMatchingDeclarationSymbol() {
        return this.getLanguageSymbol().getAllDeclarationSymbols().stream()
                .filter(symbol -> symbol.getName().equals(this.getName()))
                .findFirst();
    }
}
