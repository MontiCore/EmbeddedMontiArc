/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import java.util.*;

public class ConfigurationSymbol extends ConfigurationSymbolTOP implements SymbolWithInheritedOptions {
    protected final List<OptionFillSymbolReference> fills;
    protected final List<OptionInheritSymbolReference> inherits;

    protected ConfigurationTypeSymbolReference type;
    protected String name;
    protected Integer order;

    public ConfigurationSymbol(String name) {
        super(name);

        this.fills = new ArrayList<>();
        this.inherits = new ArrayList<>();
        this.order = null;
    }

    public void setType(ConfigurationTypeSymbolReference type) {
        this.type = type;
    }

    public Optional<ConfigurationTypeSymbolReference> getType() {
        return Optional.ofNullable(this.type);
    }

    public Optional<ConfigurationTypeSymbol> getTypeSymbol() {
        return this.getType()
                .filter(ConfigurationTypeSymbolReference::existsReferencedSymbol)
                .map(ConfigurationTypeSymbolReference::getReferencedSymbol);
    }

    public void setDisplayName(String name) {
        this.name = name;
    }

    public Optional<String> getDisplayName() {
        return Optional.ofNullable(this.name);
    }

    public void setOrder(int order) {
        this.order = order;
    }

    public Optional<Integer> getOrder() {
        return Optional.ofNullable(this.order);
    }

    @Override
    public void addOptionFill(OptionFillSymbolReference fill) {
        this.fills.add(fill);
    }

    @Override
    public List<OptionFillSymbolReference> getOptionFills() {
        return this.fills;
    }

    @Override
    public void addOptionInherit(OptionInheritSymbolReference inherit) {
        this.inherits.add(inherit);
    }

    @Override
    public List<OptionInheritSymbolReference> getOptionInherits() {
        return this.inherits;
    }
}
