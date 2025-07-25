/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import de.monticore.lang.monticar.sol.grammars.option._ast.OptionLiterals;

import java.util.Collections;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class OptionTypeSymbol extends OptionTypeSymbolTOP {
    protected final Set<PropDeclarationSymbolReference> declarations;

    protected OptionLiterals composite;
    protected String returnType;

    public OptionTypeSymbol(String name) {
        super(name);

        this.declarations = new HashSet<>();
    }

    public void setComposite(OptionLiterals composite) {
        this.composite = composite;
    }

    public Optional<OptionLiterals> getComposite() {
        return Optional.ofNullable(this.composite);
    }

    public boolean isArray() {
        return this.getComposite().map(composite -> composite == OptionLiterals.ARRAY).orElse(false);
    }

    public boolean isObject() {
        return this.getComposite().map(composite -> composite == OptionLiterals.OBJECT).orElse(false);
    }

    public boolean isComposite() {
        return this.isArray() || this.isObject();
    }

    public void setReturnType(String returnType) {
        this.returnType = returnType;
    }

    public Optional<String> getReturnType() {
        return Optional.ofNullable(this.returnType);
    }

    public void addDeclaration(PropDeclarationSymbolReference type) {
        this.declarations.add(type);
    }

    public Set<PropDeclarationSymbolReference> getDeclarations() {
        return Collections.unmodifiableSet(this.declarations);
    }

    public Set<PropDeclarationSymbol> getDeclarationSymbols() {
        return this.getDeclarations().stream()
                .filter(PropDeclarationSymbolReference::existsReferencedSymbol)
                .map(PropDeclarationSymbolReference::getReferencedSymbol)
                .collect(Collectors.toSet());
    }

    public Set<PropDeclarationSymbol> getRequiredDeclarationSymbols() {
        return this.getDeclarationSymbols().stream()
                .filter(PropDeclarationSymbol::isRequired)
                .collect(Collectors.toSet());
    }
}
