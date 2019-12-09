/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.lang.monticar.sol.grammars.option._symboltable.OptionSymbolReference;
import de.monticore.lang.monticar.sol.grammars.option._symboltable.SymbolWithOptions;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ProductSymbolReference;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbol;
import de.monticore.lang.monticar.sol.grammars.artifact._symboltable.ToolSymbolReference;

import java.util.*;
import java.util.stream.Collectors;

public class ConfigurationTypeSymbol extends ConfigurationTypeSymbolTOP
        implements SymbolWithOptions, SymbolWithConfigurations, SymbolWithTypeAttribute {
    protected final List<OptionSymbolReference> options;
    protected final List<ConfigurationSymbolReference> configurations;
    protected final List<TaskSymbolReference> tasks;
    protected final List<ToolSymbolReference> tools;
    protected final List<ProductSymbolReference> products;

    protected boolean component;
    protected String label;
    protected String icon;
    protected String category;

    public ConfigurationTypeSymbol(String name) {
        super(name);

        this.options = new ArrayList<>();
        this.configurations = new ArrayList<>();
        this.tasks = new ArrayList<>();
        this.tools = new ArrayList<>();
        this.products = new ArrayList<>();
        this.component = false;
    }

    public void setComponent(boolean component) {
        this.component = component;
    }

    public boolean isComponent() {
        return this.component;
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

    public void addTool(ToolSymbolReference tool) {
        this.tools.add(tool);
    }

    public List<ToolSymbolReference> getTools() {
        return Collections.unmodifiableList(this.tools);
    }

    public List<ToolSymbol> getToolSymbols() {
        return this.getTools().stream()
                .filter(ToolSymbolReference::existsReferencedSymbol)
                .map(ToolSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void addProduct(ProductSymbolReference product) {
        this.products.add(product);
    }

    public List<ProductSymbolReference> getProducts() {
        return Collections.unmodifiableList(this.products);
    }

    public List<ProductSymbol> getProductSymbols() {
        return this.getProducts().stream()
                .filter(ProductSymbolReference::existsReferencedSymbol)
                .map(ProductSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    @Override
    public void addOption(OptionSymbolReference option) {
        this.options.add(option);
    }

    @Override
    public List<OptionSymbolReference> getOptions() {
        return this.options;
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

    public void addTask(TaskSymbolReference task) {
        this.tasks.add(task);
    }

    public List<TaskSymbolReference> getTasks() {
        return this.tasks;
    }

    public List<TaskSymbol> getTaskSymbols() {
        return this.getTasks().stream()
                .filter(TaskSymbolReference::existsReferencedSymbol)
                .map(TaskSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public List<TaskSymbol> getFrontendTaskSymbols() {
        return this.getTaskSymbols().stream()
                .filter(TaskSymbol::isFrontend)
                .collect(Collectors.toList());
    }

    public List<TaskSymbol> getBackendTaskSymbols() {
        return this.getTaskSymbols().stream()
                .filter(TaskSymbol::isBackend)
                .collect(Collectors.toList());
    }
}
