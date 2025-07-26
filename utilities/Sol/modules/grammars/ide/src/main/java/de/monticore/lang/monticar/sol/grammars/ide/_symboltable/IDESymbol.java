/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._ast.CommonLiterals;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.SymbolWithEnvironment;
import de.monticore.lang.monticar.sol.runtime.grammar.symboltable.HeritageSet;

import java.util.*;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class IDESymbol extends IDESymbolTOP {
    protected final List<IDESymbolReference> parents;
    protected final HeritageSet<ModuleTypeSymbolReference> moduleTypes;
    protected final HeritageSet<ConfigurationTypeSymbolReference> configurationTypes;

    protected boolean online;
    protected String registry;
    protected String buildPath;
    protected CommonLiterals buildOrigin;

    public IDESymbol(String name) {
        super(name);

        this.parents = new ArrayList<>();
        this.online = false;
        this.moduleTypes = new HeritageSet<>();
        this.configurationTypes = new HeritageSet<>();
    }

    public void setOnline(boolean online) {
        this.online = online;
    }

    public boolean isOnline() {
        return this.online;
    }

    public void setRegistry(String registry) {
        this.registry = registry;
    }

    public Optional<String> getRegistry() {
        return Optional.ofNullable(this.registry);
    }

    public void setBuildPath(String buildPath) {
        this.buildPath = buildPath;
    }

    public Optional<String> getBuildPath() {
        return Optional.ofNullable(this.buildPath);
    }

    public void setBuildOrigin(CommonLiterals buildOrigin) {
        this.buildOrigin = buildOrigin;
    }

    public Optional<CommonLiterals> getBuildOrigin() {
        return Optional.ofNullable(this.buildOrigin);
    }

    public void addParent(IDESymbolReference parent) {
        this.parents.add(parent);
    }

    public List<IDESymbolReference> getParents() {
        return Collections.unmodifiableList(this.parents);
    }

    public List<IDESymbol> getParentSymbols() {
        return this.getParents().stream()
                .filter(IDESymbolReference::existsReferencedSymbol)
                .map(IDESymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void includeType(ModuleTypeSymbolReference moduleType) {
        this.moduleTypes.include(moduleType);
    }

    public void excludeType(ModuleTypeSymbolReference moduleType) {
        this.moduleTypes.exclude(moduleType);
    }

    public void includeType(ConfigurationTypeSymbolReference configurationType) {
        this.configurationTypes.include(configurationType);
    }

    public void excludeType(ConfigurationTypeSymbolReference configurationType) {
        this.configurationTypes.exclude(configurationType);
    }

    public Set<ModuleTypeSymbolReference> getModuleTypeInclusions() {
        return new HashSet<>(this.moduleTypes.getInclusions());
    }

    public Set<ModuleTypeSymbol> getModuleTypeInclusionSymbols() {
        return this.getModuleTypeSymbols(this.getModuleTypeInclusions());
    }

    public Set<ModuleTypeSymbolReference> getModuleTypeExclusions() {
        return new HashSet<>(this.moduleTypes.getExclusions());
    }

    public Set<ModuleTypeSymbol> getModuleTypeExclusionSymbols() {
        return this.getModuleTypeSymbols(this.getModuleTypeExclusions());
    }

    public Set<ModuleTypeSymbol> getLocalNeededModuleTypeSymbols() {
        Set<ModuleTypeSymbol> inclusions = this.getModuleTypeInclusionSymbols();
        Set<ModuleTypeSymbol> needed = new HashSet<>(inclusions);

        inclusions.stream()
                .flatMap(module -> module.getAllModuleTypeSymbols().stream())
                .forEach(needed::add);

        return needed;
    }

    public Set<ModuleTypeSymbol> getAllNeededModuleTypeSymbols() {
        Set<ModuleTypeSymbol> localNeeded = this.getLocalNeededModuleTypeSymbols();
        Set<ModuleTypeSymbol> result = new HashSet<>(localNeeded);
        Set<ModuleTypeSymbol> inheritedNeeded = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllNeededModuleTypeSymbols().stream())
                .collect(Collectors.toSet());

        result.addAll(inheritedNeeded);
        return result;
    }

    public Set<ModuleTypeSymbol> getInheritedModuleTypeSymbols() {
        return this.getParentSymbols().stream()
                .flatMap(parent -> parent.getEffectiveModuleTypeSymbols().stream())
                .collect(Collectors.toSet());
    }

    public Set<ModuleTypeSymbol> getEffectiveModuleTypeSymbols() {
        Set<ModuleTypeSymbol> inclusions = this.getInheritedModuleTypeSymbols();
        Set<String> exclusions = this.getModuleTypeExclusionSymbols().stream()
                .map(ModuleTypeSymbol::getFullName)
                .collect(Collectors.toSet());

        inclusions.addAll(this.getModuleTypeInclusionSymbols());

        return inclusions.stream()
                .filter(symbol -> !exclusions.contains(symbol.getFullName()))
                .collect(Collectors.toSet());
    }

    public Set<ModuleTypeSymbol> getAllModuleTypeSymbols() {
        Set<ModuleTypeSymbol> inclusions = this.getModuleTypeInclusionSymbols();
        Set<ModuleTypeSymbol> result = new HashSet<>(inclusions);

        inclusions.stream()
                .flatMap(module -> module.getAllModuleTypeSymbols().stream())
                .forEach(result::add);

        return result;
    }

    protected Set<ModuleTypeSymbol> getModuleTypeSymbols(Set<ModuleTypeSymbolReference> references) {
        return references.stream()
                .filter(ModuleTypeSymbolReference::existsReferencedSymbol)
                .map(ModuleTypeSymbolReference::getReferencedSymbol)
                .collect(Collectors.toSet());
    }

    public Set<ConfigurationTypeSymbolReference> getConfigurationTypeInclusions() {
        return new HashSet<>(this.configurationTypes.getInclusions());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeInclusionSymbols() {
        return this.getConfigurationTypeSymbols(this.getConfigurationTypeInclusions());
    }

    public Set<ConfigurationTypeSymbol> getLocalNeededConfigurationTypeSymbols() {
        Set<ConfigurationTypeSymbol> inclusions = this.getConfigurationTypeInclusionSymbols();
        Set<ConfigurationTypeSymbol> result = new HashSet<>(inclusions);
        Set<ConfigurationTypeSymbol> moduleConfigurations = this.getLocalNeededModuleTypeSymbols().stream()
                .flatMap(module -> module.getConfigurationTypeSymbols().stream())
                .collect(Collectors.toSet());
        Set<ConfigurationTypeSymbol> configurationConfigurations = inclusions.stream()
                .flatMap(configuration -> configuration.getAllConfigurationTypeSymbols().stream())
                .collect(Collectors.toSet());

        result.addAll(moduleConfigurations);
        result.addAll(configurationConfigurations);
        return result;
    }

    public Set<ConfigurationTypeSymbol> getAllNeededConfigurationTypeSymbols() {
        Set<ConfigurationTypeSymbol> localNeeded = this.getLocalNeededConfigurationTypeSymbols();
        Set<ConfigurationTypeSymbol> result = new HashSet<>(localNeeded);
        Set<ConfigurationTypeSymbol> inheritedNeeded = this.getParentSymbols().stream()
                .flatMap(parent -> parent.getAllNeededConfigurationTypeSymbols().stream())
                .collect(Collectors.toSet());

        result.addAll(inheritedNeeded);
        return result;
    }

    public Set<ConfigurationTypeSymbol> getInheritedConfigurationTypeSymbols() {
        return this.getParentSymbols().stream()
                .flatMap(parent -> parent.getEffectiveConfigurationTypeSymbols().stream())
                .collect(Collectors.toSet());
    }

    public Set<ConfigurationTypeSymbol> getEffectiveConfigurationTypeSymbols() {
        Set<ConfigurationTypeSymbol> inclusions = this.getInheritedConfigurationTypeSymbols();
        Set<String> exclusions = this.getConfigurationTypeExclusionSymbols().stream()
                .map(ConfigurationTypeSymbol::getFullName)
                .collect(Collectors.toSet());

        inclusions.addAll(this.getConfigurationTypeInclusionSymbols());

        return inclusions.stream()
                .filter(symbol -> !exclusions.contains(symbol.getFullName()))
                .collect(Collectors.toSet());
    }

    public Set<ConfigurationTypeSymbolReference> getConfigurationTypeExclusions() {
        return new HashSet<>(this.configurationTypes.getExclusions());
    }

    public Set<ConfigurationTypeSymbol> getConfigurationTypeExclusionSymbols() {
        return this.getConfigurationTypeSymbols(this.getConfigurationTypeExclusions());
    }

    public Set<ConfigurationTypeSymbol> getAllConfigurationTypeSymbols() {
        Set<ConfigurationTypeSymbol> cinclusions = this.getConfigurationTypeInclusionSymbols();
        Set<ModuleTypeSymbol> minclusions = this.getAllModuleTypeSymbols();
        Set<ConfigurationTypeSymbol> result = new HashSet<>(cinclusions);

        cinclusions.stream()
                .flatMap(configuration -> configuration.getAllConfigurationTypeSymbols().stream())
                .forEach(result::add);
        minclusions.stream()
                .flatMap(module -> module.getAllConfigurationTypeSymbols().stream())
                .forEach(result::add);

        return result;
    }

    protected Set<ConfigurationTypeSymbol> getConfigurationTypeSymbols(Set<ConfigurationTypeSymbolReference> references) {
        return references.stream()
                .filter(ConfigurationTypeSymbolReference::existsReferencedSymbol)
                .map(ConfigurationTypeSymbolReference::getReferencedSymbol)
                .collect(Collectors.toSet());
    }

    public Set<DockerfileSymbol> getEnvironmentSymbols() {
        Set<DockerfileSymbol> allEnvironments = new HashSet<>();
        Set<ConfigurationTypeSymbol> configurations = this.getEffectiveConfigurationTypeSymbols();
        Stream<SymbolWithEnvironment> tools = configurations.stream()
                .flatMap(configuration -> configuration.getToolSymbols().stream());
        Stream<SymbolWithEnvironment> products = configurations.stream()
                .flatMap(configuration -> configuration.getProductSymbols().stream());

        allEnvironments.addAll(this.getEnvironmentSymbols(tools));
        allEnvironments.addAll(this.getEnvironmentSymbols(products));
        return allEnvironments;
    }

    protected Set<DockerfileSymbol> getEnvironmentSymbols(Stream<SymbolWithEnvironment> stream) {
        return stream
                .map(SymbolWithEnvironment::getEnvironmentSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toSet());
    }

    public Set<Integer> getPorts() {
        Set<DockerfileSymbol> allEnvironments = this.getEnvironmentSymbols();

        return allEnvironments.stream()
                .flatMap(environment -> environment.getAllPorts().stream())
                .collect(Collectors.toSet());
    }
}
