/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralFillSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.fill.LiteralListFillSymbol;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbol;

import java.util.Optional;
import java.util.function.Predicate;

public class OptionFillSymbol extends OptionFillSymbolTOP {
    protected ConfigurationSymbolReference owner;

    public OptionFillSymbol(String name) {
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

    public boolean isLiteralFill() {
        return this instanceof LiteralFillSymbol;
    }

    public Optional<LiteralFillSymbol> asLiteralFill() {
        return this.isLiteralFill() ? Optional.of((LiteralFillSymbol) this) : Optional.empty();
    }

    public boolean isLiteralListFill() {
        return this instanceof LiteralListFillSymbol;
    }

    public Optional<LiteralListFillSymbol> asLiteralListFill() {
        return this.isLiteralListFill() ? Optional.of((LiteralListFillSymbol) this) : Optional.empty();
    }
}
