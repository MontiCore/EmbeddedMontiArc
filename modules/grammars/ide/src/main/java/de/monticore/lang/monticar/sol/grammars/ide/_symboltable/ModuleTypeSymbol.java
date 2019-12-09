/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;


import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;

import java.util.*;
import java.util.stream.Collectors;

public class ModuleTypeSymbol extends ModuleTypeSymbolTOP
        implements SymbolWithOptions, SymbolWithConfigurations, SymbolWithTypeAttribute {
    protected final List<OptionSymbolReference> options;
    protected final List<ModuleSymbolReference> modules;
    protected final List<ConfigurationSymbolReference> configurations;
    protected final List<WriteSymbolReference> writes;

    protected String label;
    protected String icon;
    protected String category;

    public ModuleTypeSymbol(String name) {
        super(name);

        this.options = new ArrayList<>();
        this.modules = new ArrayList<>();
        this.configurations = new ArrayList<>();
        this.writes = new ArrayList<>();
    }

    @Override
    public void setLabel(String label) {
        this.label = label;
    }

    @Override
    public String getLabel() {
        return Optional.ofNullable(this.label).orElse("Unknown Label");
    }

    @Override
    public void setIcon(String icon) {
        this.icon = icon;
    }

    @Override
    public Optional<String> getIcon() {
        return Optional.ofNullable(this.icon);
    }

    @Override
    public void setCategory(String category) {
        this.category = category;
    }

    @Override
    public Optional<String> getCategory() {
        return Optional.ofNullable(this.category);
    }

    @Override
    public void addOption(OptionSymbolReference option) {
        this.options.add(option);
    }

    @Override
    public List<OptionSymbolReference> getOptions() {
        return Collections.unmodifiableList(this.options);
    }

    public void addModule(ModuleSymbolReference module) {
        this.modules.add(module);
    }

    public List<ModuleSymbolReference> getModules() {
        return Collections.unmodifiableList(this.modules);
    }

    public List<ModuleSymbol> getModuleSymbols() {
        return this.getModules().stream()
                .filter(ModuleSymbolReference::existsReferencedSymbol)
                .map(ModuleSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<ModuleTypeSymbol> getModuleTypeSymbols() {
        return this.getModuleSymbols().stream()
                .map(ModuleSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());
    }

    public List<ModuleTypeSymbol> getAllModuleTypeSymbols() {
        List<ModuleTypeSymbol> symbols = this.getModuleTypeSymbols();
        List<ModuleTypeSymbol> allSymbols = new ArrayList<>(symbols);

        symbols.stream()
                .flatMap(module -> module.getAllModuleTypeSymbols().stream())
                .forEach(allSymbols::add);

        return allSymbols;
    }

    @Override
    public void addConfiguration(ConfigurationSymbolReference configuration) {
        this.configurations.add(configuration);
    }

    @Override
    public List<ConfigurationSymbolReference> getConfigurations() {
        return Collections.unmodifiableList(this.configurations);
    }

    @Override
    public List<ConfigurationSymbol> getConfigurationSymbols() {
        return this.getConfigurations().stream()
                .filter(ConfigurationSymbolReference::existsReferencedSymbol)
                .map(ConfigurationSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addWrite(WriteSymbolReference write) {
        this.writes.add(write);
    }

    public List<WriteSymbolReference> getWrites() {
        return Collections.unmodifiableList(this.writes);
    }

    public List<WriteSymbol> getWriteSymbols() {
        return this.getWrites().stream()
                .filter(WriteSymbolReference::existsReferencedSymbol)
                .map(WriteSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<WriteSymbol> getArtifactWriteSymbols() {
        return this.getWriteSymbols().stream()
                .filter(WriteSymbol::isReferencingArtifact)
                .collect(Collectors.toList());
    }

    public List<WriteSymbol> getModuleWriteSymbols() {
        return this.getWriteSymbols().stream()
                .filter(WriteSymbol::isReferencingModule)
                .collect(Collectors.toList());
    }
}
