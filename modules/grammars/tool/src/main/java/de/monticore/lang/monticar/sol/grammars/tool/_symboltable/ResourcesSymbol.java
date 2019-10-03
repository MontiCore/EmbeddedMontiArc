/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class ResourcesSymbol extends ResourcesSymbolTOP implements RootSymbol {
    protected final List<AttributeSymbolReference> attributes;
    protected final List<ResourceSymbolReference> resources;

    public ResourcesSymbol(String name) {
        super(name);

        this.attributes = new ArrayList<>();
        this.resources = new ArrayList<>();
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

    public void addResource(ResourceSymbolReference resource) {
        this.resources.add(resource);
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
