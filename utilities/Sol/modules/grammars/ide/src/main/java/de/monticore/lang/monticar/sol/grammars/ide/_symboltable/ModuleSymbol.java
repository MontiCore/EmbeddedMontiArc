/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import java.util.*;

public class ModuleSymbol extends ModuleSymbolTOP implements SymbolWithFilledOptions {
    protected final List<OptionFillSymbolReference> fills;
    protected final List<OptionInheritSymbolReference> inherits;

    protected ModuleTypeSymbolReference type;

    public ModuleSymbol(String name) {
        super(name);

        this.fills = new ArrayList<>();
        this.inherits = new ArrayList<>();
    }

    public void setType(ModuleTypeSymbolReference type) {
        this.type = type;
    }

    public Optional<ModuleTypeSymbolReference> getType() {
        return Optional.ofNullable(this.type);
    }

    public Optional<ModuleTypeSymbol> getTypeSymbol() {
        return this.getType()
                .filter(ModuleTypeSymbolReference::existsReferencedSymbol)
                .map(ModuleTypeSymbolReference::getReferencedSymbol);
    }

    @Override
    public void addOptionFill(OptionFillSymbolReference fill) {
        this.fills.add(fill);
    }

    @Override
    public List<OptionFillSymbolReference> getOptionFills() {
        return this.fills;
    }
}
