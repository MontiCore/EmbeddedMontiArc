/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionSymbol;
import de.monticore.lang.monticar.sol.grammars.options._symboltable.OptionSymbolReference;

import java.util.*;
import java.util.stream.Collectors;

public class TemplateDeclarationSymbol extends TemplateDeclarationSymbolTOP {
    protected final List<TemplateAttributeSymbolReference> attributes;
    protected final List<OptionSymbolReference> options;

    protected String path;

    public TemplateDeclarationSymbol(String name) {
        super(name);

        this.attributes = new ArrayList<>();
        this.options = new ArrayList<>();
    }

    public void addAttribute(TemplateAttributeSymbolReference attribute) {
        this.attributes.add(attribute);
    }

    public List<TemplateAttributeSymbolReference> getAttributes() {
        return Collections.unmodifiableList(this.attributes);
    }

    public List<TemplateAttributeSymbol> getAttributeSymbols() {
        return this.getAttributes().stream()
                .filter(TemplateAttributeSymbolReference::existsReferencedSymbol)
                .map(TemplateAttributeSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addOption(OptionSymbolReference option) {
        this.options.add(option);
    }

    public List<OptionSymbolReference> getOptions() {
        return Collections.unmodifiableList(this.options);
    }

    public List<OptionSymbol> getOptionSymbols() {
        return this.getOptions().stream()
                .filter(OptionSymbolReference::existsReferencedSymbol)
                .map(OptionSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void setPath(String path) {
        this.path = path;
    }

    public String getPath() {
        return this.path;
    }
}
