/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class ToolSymbol extends ToolSymbolTOP implements RootSymbol {
    protected final List<AttributeSymbolReference> attributes;
    protected final List<ResourceSymbolReference> resources;

    protected boolean virtual;

    public ToolSymbol(String name) {
        super(name);

        this.attributes = new ArrayList<>();
        this.resources = new ArrayList<>();
    }

    public boolean isVirtual() {
        return this.virtual;
    }

    public void setVirtual(boolean virtual) {
        this.virtual = virtual;
    }

    public void addAttribute(AttributeSymbolReference attribute) {
        this.attributes.add(attribute);
    }

    public List<AttributeSymbolReference> getAttributes() {
        return Collections.unmodifiableList(this.attributes);
    }

    public List<AttributeSymbol> getAttributeSymbols() {
        return this.getAttributes().stream()
                .filter(AttributeSymbolReference::existsReferencedSymbol)
                .map(AttributeSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addResource(ResourceSymbolReference attribute) {
        this.resources.add(attribute);
    }

    public List<ResourceSymbolReference> getResources() {
        return Collections.unmodifiableList(this.resources);
    }

    public List<ResourceSymbol> getResourceSymbols() {
        return this.getResources().stream()
                .filter(ResourceSymbolReference::existsReferencedSymbol)
                .map(ResourceSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }
}
