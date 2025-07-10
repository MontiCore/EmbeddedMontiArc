/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide.cocos;

import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTConfigurationType;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTIDE;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTModuleType;
import de.monticore.lang.monticar.sol.grammars.ide._ast.ASTTask;
import de.monticore.lang.monticar.sol.grammars.ide._cocos.*;
import de.monticore.lang.monticar.sol.grammars.ide._symboltable.*;

import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

public class CircularDependencyCoCo extends CommonIDECoCo
        implements IDEASTIDECoCo, IDEASTTaskCoCo, IDEASTModuleTypeCoCo, IDEASTConfigurationTypeCoCo {
    public CircularDependencyCoCo() {
        super("IDE0006", "Circular dependency detected for %s '%s'.");
    }

    @Override
    public void registerTo(IDECoCoChecker checker) {
        checker.addCoCo((IDEASTIDECoCo) this);
        checker.addCoCo((IDEASTTaskCoCo) this);
        checker.addCoCo((IDEASTModuleTypeCoCo) this);
        checker.addCoCo((IDEASTConfigurationTypeCoCo) this);
    }

    @Override
    public void check(ASTIDE node) {
        node.getIDESymbolOpt().ifPresent(this::check);
    }

    protected void check(IDESymbol language) {
        this.check(language, language.getParentSymbols());
    }

    protected void check(IDESymbol language, List<IDESymbol> predecessors) {
        String fullName = language.getFullName();
        Optional<IDESymbol> filteredLanguage = predecessors.stream()
                .filter(predecessor -> predecessor.getFullName().equals(fullName))
                .findFirst();

        if (filteredLanguage.isPresent()) this.error(language, "ide", fullName);
        else this.nextGeneration(language, predecessors);
    }

    protected void nextGeneration(IDESymbol language, List<IDESymbol> predecessors) {
        List<IDESymbol> nextPredecessors = predecessors.stream()
                .flatMap(predecessor -> predecessor.getParentSymbols().stream())
                .collect(Collectors.toList());

        if (!nextPredecessors.isEmpty()) this.check(language, nextPredecessors);
    }

    @Override
    public void check(ASTTask node) {
        node.getTaskSymbolOpt().ifPresent(this::check);
    }

    protected void check(TaskSymbol task) {
        this.check(task, task.getPredecessorSymbols());
    }

    protected void check(TaskSymbol task, List<TaskSymbol> predecessors) {
        String fullName = task.getFullName();
        Optional<TaskSymbol> filteredTask = predecessors.stream()
                .filter(predecessor -> predecessor.getFullName().equals(fullName))
                .findFirst();

        if (filteredTask.isPresent()) this.error(task, "task", fullName);
        else this.nextGeneration(task, predecessors);
    }

    protected void nextGeneration(TaskSymbol task, List<TaskSymbol> predecessors) {
        List<TaskSymbol> nextPredecessors = predecessors.stream()
                .flatMap(predecessor -> predecessor.getPredecessorSymbols().stream())
                .collect(Collectors.toList());

        if (!nextPredecessors.isEmpty()) this.check(task, nextPredecessors);
    }

    @Override
    public void check(ASTConfigurationType node) {
        node.getConfigurationTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ConfigurationTypeSymbol symbol) {
        List<ConfigurationTypeSymbol> types = symbol.getConfigurationSymbols().stream()
                .map(ConfigurationSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());

        this.check(symbol, types);
    }

    protected void check(ConfigurationTypeSymbol symbol, List<ConfigurationTypeSymbol> components) {
        String fullName = symbol.getFullName();
        Optional<ConfigurationTypeSymbol> filteredTask = components.stream()
                .filter(component -> component.getFullName().equals(fullName))
                .findFirst();

        if (filteredTask.isPresent()) this.error(symbol, "configuration type", fullName);
        else this.nextGeneration(symbol, components);
    }

    protected void nextGeneration(ConfigurationTypeSymbol task, List<ConfigurationTypeSymbol> components) {
        List<ConfigurationTypeSymbol> nextComponents = components.stream()
                .flatMap(component -> component.getConfigurationSymbols().stream())
                .map(ConfigurationSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());

        if (!nextComponents.isEmpty()) this.check(task, nextComponents);
    }

    @Override
    public void check(ASTModuleType node) {
        node.getModuleTypeSymbolOpt().ifPresent(this::check);
    }

    protected void check(ModuleTypeSymbol symbol) {
        List<ModuleTypeSymbol> types = symbol.getModuleSymbols().stream()
                .map(ModuleSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());

        this.check(symbol, types);
    }

    protected void check(ModuleTypeSymbol symbol, List<ModuleTypeSymbol> components) {
        String fullName = symbol.getFullName();
        Optional<ModuleTypeSymbol> filteredTask = components.stream()
                .filter(component -> component.getFullName().equals(fullName))
                .findFirst();

        if (filteredTask.isPresent()) this.error(symbol, "module type", fullName);
        else this.nextGeneration(symbol, components);
    }

    protected void nextGeneration(ModuleTypeSymbol task, List<ModuleTypeSymbol> components) {
        List<ModuleTypeSymbol> nextComponents = components.stream()
                .flatMap(component -> component.getModuleSymbols().stream())
                .map(ModuleSymbol::getTypeSymbol)
                .filter(Optional::isPresent)
                .map(Optional::get)
                .collect(Collectors.toList());

        if (!nextComponents.isEmpty()) this.check(task, nextComponents);
    }
}
