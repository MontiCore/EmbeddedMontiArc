/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class ResourceSymbol extends ResourceSymbolTOP implements AttributableSymbol {
    protected final List<AttributeSymbolReference> attributes;

    public ResourceSymbol(String name) {
        super(name);

        this.attributes = new ArrayList<>();
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
}
