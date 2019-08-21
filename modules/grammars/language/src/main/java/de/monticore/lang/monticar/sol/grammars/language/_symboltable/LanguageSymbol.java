/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import java.util.*;
import java.util.stream.Collectors;

public class LanguageSymbol extends LanguageSymbolTOP {
    protected final List<LanguageSymbolReference> parents;
    protected final List<TemplateDeclarationSymbolReference> declarations;
    protected final List<TemplateUndeclarationSymbolReference> undeclarations;

    public LanguageSymbol(String name) {
        super(name);

        this.parents = new ArrayList<>();
        this.declarations = new ArrayList<>();
        this.undeclarations = new ArrayList<>();
    }

    public void addParent(LanguageSymbolReference parent) {
        this.parents.add(parent);
    }

    public List<LanguageSymbolReference> getParents() {
        return Collections.unmodifiableList(this.parents);
    }

    public List<LanguageSymbol> getParentSymbols() {
        return this.getParents().stream()
                .filter(LanguageSymbolReference::existsReferencedSymbol)
                .map(LanguageSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addLocalDeclaration(TemplateDeclarationSymbolReference declaration) {
        this.declarations.add(declaration);
    }

    public List<TemplateDeclarationSymbolReference> getLocalDeclarations() {
        return Collections.unmodifiableList(this.declarations);
    }

    public List<TemplateDeclarationSymbol> getLocalDeclarationSymbols() {
        return this.getLocalDeclarations().stream()
                .filter(TemplateDeclarationSymbolReference::existsReferencedSymbol)
                .map(TemplateDeclarationSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<TemplateDeclarationSymbol> getAllDeclarationSymbols() {
        List<TemplateDeclarationSymbol> symbols = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllDeclarationSymbols().stream())
                .collect(Collectors.toList());

        symbols.addAll(this.getLocalDeclarationSymbols());

        return symbols;
    }

    public void addLocalUndeclaration(TemplateUndeclarationSymbolReference undeclaration) {
        this.undeclarations.add(undeclaration);
    }

    public List<TemplateUndeclarationSymbolReference> getLocalUndeclarations() {
        return Collections.unmodifiableList(this.undeclarations);
    }

    public List<TemplateUndeclarationSymbol> getLocalUndeclarationSymbols() {
        return this.getLocalUndeclarations().stream()
                .filter(TemplateUndeclarationSymbolReference::existsReferencedSymbol)
                .map(TemplateUndeclarationSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<TemplateUndeclarationSymbol> getAllUndeclarationSymbols() {
        List<TemplateUndeclarationSymbol> symbols = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllUndeclarationSymbols().stream())
                .collect(Collectors.toList());

        symbols.addAll(this.getLocalUndeclarationSymbols());

        return symbols;
    }
}
