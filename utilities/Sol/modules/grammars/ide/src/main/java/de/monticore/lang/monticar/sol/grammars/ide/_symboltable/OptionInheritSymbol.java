/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;


import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;

import java.util.Optional;
import java.util.function.Predicate;

public class OptionInheritSymbol extends OptionInheritSymbolTOP {
    protected ConfigurationSymbolReference owner;
    protected OptionSymbolReference parent;

    public OptionInheritSymbol(String name) {
        super(name);
    }

    public void setOwner(ConfigurationSymbolReference owner) {
        this.owner = owner;
    }

    public Optional<ConfigurationSymbolReference> getOwner() {
        return Optional.ofNullable(this.owner);
    }

    public Optional<ConfigurationSymbol> getOwnerSymbol() {
        return this.getOwner()
                .filter(ConfigurationSymbolReference::existsReferencedSymbol)
                .map(ConfigurationSymbolReference::getReferencedSymbol);
    }

    public Optional<OptionSymbol> getOptionSymbol() {
        Predicate<OptionSymbol> predicate = option -> option.getName().equals(this.getName());

        return this.getOwnerSymbol()
                .flatMap(ConfigurationSymbol::getTypeSymbol)
                .flatMap(configuration -> configuration.getOptionSymbols().stream().filter(predicate).findFirst());
    }

    public void setParent(OptionSymbolReference parent) {
        this.parent = parent;
    }

    public Optional<OptionSymbolReference> getParent() {
        return Optional.ofNullable(this.parent);
    }

    public Optional<OptionSymbol> getParentSymbol() {
        return this.getSymbolOfReference(this.parent);
    }

    protected Optional<OptionSymbol> getSymbolOfReference(OptionSymbolReference reference) {
        return Optional.ofNullable(reference)
                .filter(OptionSymbolReference::existsReferencedSymbol)
                .map(OptionSymbolReference::getReferencedSymbol);
    }
}
