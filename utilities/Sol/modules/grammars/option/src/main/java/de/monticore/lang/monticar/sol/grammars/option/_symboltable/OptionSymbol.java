/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import java.util.*;
import java.util.stream.Collectors;

public class OptionSymbol extends OptionSymbolTOP implements SymbolWithOptions {
    protected final Set<PropAssignmentSymbolReference> props;
    protected final List<OptionSymbolReference> options;

    protected OptionTypeSymbolReference type;

    public OptionSymbol(String name) {
        super(name);

        this.props = new HashSet<>();
        this.options = new ArrayList<>();
    }

    public void setType(OptionTypeSymbolReference type) {
        this.type = type;
    }

    public Optional<OptionTypeSymbolReference> getType() {
        return Optional.ofNullable(this.type);
    }

    public Optional<OptionTypeSymbol> getTypeSymbol() {
        return this.getType()
                .filter(OptionTypeSymbolReference::existsReferencedSymbol)
                .map(OptionTypeSymbolReference::getReferencedSymbol);
    }

    public void addAssignment(PropAssignmentSymbolReference prop) {
        this.props.add(prop);
    }

    public Set<PropAssignmentSymbolReference> getAssignments() {
        return Collections.unmodifiableSet(this.props);
    }

    public Set<PropAssignmentSymbol> getAssignmentSymbols() {
        return this.getAssignments().stream()
                .filter(PropAssignmentSymbolReference::existsReferencedSymbol)
                .map(PropAssignmentSymbolReference::getReferencedSymbol)
                .collect(Collectors.toSet());
    }

    @Override
    public void addOption(OptionSymbolReference option) {
        this.options.add(option);
    }

    @Override
    public List<OptionSymbolReference> getOptions() {
        return Collections.unmodifiableList(this.options);
    }
}
