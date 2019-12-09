/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralAssignmentSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.assignment.LiteralListAssignmentSymbol;

import java.util.HashSet;
import java.util.Optional;

public class PropAssignmentSymbol extends PropAssignmentSymbolTOP {
    protected OptionSymbolReference owner;

    public PropAssignmentSymbol(String name) {
        super(name);
    }

    public void setOwner(OptionSymbolReference owner) {
        this.owner = owner;
    }

    public Optional<OptionSymbolReference> getOwner() {
        return Optional.ofNullable(this.owner);
    }

    public Optional<OptionSymbol> getOwnerSymbol() {
        return this.getOwner()
                .filter(OptionSymbolReference::existsReferencedSymbol)
                .map(OptionSymbolReference::getReferencedSymbol);
    }

    public Optional<PropDeclarationSymbolReference> getDeclaration() {
        return this.getOwnerSymbol()
                .flatMap(OptionSymbol::getTypeSymbol)
                .map(OptionTypeSymbol::getDeclarations)
                .orElse(new HashSet<>()).stream()
                .filter(declaration -> declaration.getName().equals(this.getName()))
                .findFirst();
    }

    public Optional<PropDeclarationSymbol> getDeclarationSymbol() {
        return this.getDeclaration()
                .filter(PropDeclarationSymbolReference::existsReferencedSymbol)
                .map(PropDeclarationSymbolReference::getReferencedSymbol);
    }

    public boolean isLiteralAssignment() {
        return this instanceof LiteralAssignmentSymbol;
    }

    public Optional<LiteralAssignmentSymbol> asLiteralAssignment() {
        return this.isLiteralAssignment() ? Optional.of((LiteralAssignmentSymbol)this) : Optional.empty();
    }

    public boolean isLiteralListAssignment() {
        return this instanceof LiteralListAssignmentSymbol;
    }

    public Optional<LiteralListAssignmentSymbol> asLiteralListAssignment() {
        return this.isLiteralListAssignment() ? Optional.of((LiteralListAssignmentSymbol)this) : Optional.empty();
    }
}
