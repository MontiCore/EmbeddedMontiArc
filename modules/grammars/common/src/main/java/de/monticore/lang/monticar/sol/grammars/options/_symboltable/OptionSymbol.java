/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.options._symboltable;

import java.util.*;
import java.util.stream.Collectors;

public class OptionSymbol extends OptionSymbolTOP {
    protected final List<OptionPropSymbolReference> props;
    protected final List<OptionSymbolReference> subOptions;

    protected OptionSymbolReference parent;

    public OptionSymbol(String name) {
        super(name);

        this.props = new ArrayList<>();
        this.subOptions = new ArrayList<>();
    }

    public Optional<OptionSymbolReference> getParent() {
        return Optional.ofNullable(this.parent);
    }

    public Optional<OptionSymbol> getParentSymbol() {
        return this.getParent().isPresent() ?
                Optional.ofNullable(this.getParent().get().getReferencedSymbol()) :
                Optional.empty();
    }

    public void addProp(OptionPropSymbolReference prop) {
        this.props.add(prop);
    }

    public List<OptionPropSymbolReference> getProps() {
        return Collections.unmodifiableList(this.props);
    }

    public List<OptionPropSymbol> getPropSymbols() {
        return this.getProps().stream()
                .filter(OptionPropSymbolReference::existsReferencedSymbol)
                .map(OptionPropSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addSubOption(OptionSymbolReference option) {
        this.subOptions.add(option);
    }

    public List<OptionSymbolReference> getSubOptions() {
        return Collections.unmodifiableList(this.subOptions);
    }

    public List<OptionSymbol> getSubOptionSymbols() {
        return this.getSubOptions().stream()
                .filter(OptionSymbolReference::existsReferencedSymbol)
                .map(OptionSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
