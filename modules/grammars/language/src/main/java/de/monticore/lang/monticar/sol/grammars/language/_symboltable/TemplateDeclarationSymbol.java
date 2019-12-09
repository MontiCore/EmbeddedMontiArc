/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithPath;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

public class TemplateDeclarationSymbol extends TemplateDeclarationSymbolTOP implements SymbolWithPath, SymbolWithOptions {
    protected final List<OptionSymbolReference> options;

    protected String path;
    protected String label;

    public TemplateDeclarationSymbol(String name) {
        super(name);

        this.options = new ArrayList<>();
    }

    @Override
    public void addOption(OptionSymbolReference option) {
        this.options.add(option);
    }

    @Override
    public List<OptionSymbolReference> getOptions() {
        return Collections.unmodifiableList(this.options);
    }

    @Override
    public void setPath(String path) {
        this.path = path;
    }

    @Override
    public Optional<String> getPath() {
        return Optional.ofNullable(this.path);
    }

    public void setLabel(String label) {
        this.label = label;
    }

    public Optional<String> getLabel() {
        return Optional.ofNullable(this.label);
    }
}
