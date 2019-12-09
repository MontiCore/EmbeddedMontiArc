/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.plugins.ide.plugin.symboltable;

import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ConfigurationTypeSymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.IDESymbol;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.ModuleTypeSymbol;
import de.monticore.lang.monticar.sol.plugins.ide.plugin.generator.common.filter.LocalFilter;

import java.util.Set;
import java.util.stream.Collectors;

public class LocalAwareIDESymbol {
    protected final IDESymbol core;
    protected final LocalFilter filter;

    public LocalAwareIDESymbol(IDESymbol core, LocalFilter filter) {
        this.core = core;
        this.filter = filter;
    }

    public String getName() {
        return this.core.getName();
    }

    public String getPackageName() {
        return this.core.getPackageName();
    }

    public String getFullName() {
        return this.core.getFullName();
    }

    public Set<ModuleTypeSymbol> getModuleTypeInclusionSymbols() {
        return this.filter.filter(this.core.getAllModuleTypeSymbols());
    }

    public Set<ModuleTypeSymbol> getModuleTypeExclusionSymbols() {
        return this.filter.filter(this.core.getModuleTypeExclusionSymbols());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeInclusionSymbols(boolean component) {
        return this.filter.filter(this.core.getAllConfigurationTypeSymbols()).stream()
                .filter(configuration -> configuration.isComponent() == component)
                .collect(Collectors.toSet());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeInclusionSymbols() {
        return this.filter.filter(this.core.getAllConfigurationTypeSymbols());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeExclusionSymbols(boolean component) {
        return this.filter.filter(this.core.getConfigurationTypeExclusionSymbols()).stream()
                .filter(configuration -> configuration.isComponent() == component)
                .collect(Collectors.toSet());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeExclusionSymbols() {
        return this.filter.filter(this.core.getConfigurationTypeExclusionSymbols());
    }

    public Set<DockerfileSymbol> getEnvironmentSymbols() {
        return this.core.getEnvironmentSymbols();
    }

    public Set<Integer> getPorts() {
        return this.core.getPorts();
    }
}
