/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.ide._symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class TaskSymbol extends TaskSymbolTOP {
    protected final List<TaskSymbolReference> predecessors;

    protected boolean frontend;
    protected boolean backend;

    public TaskSymbol(String name) {
        super(name);

        this.predecessors = new ArrayList<>();
    }

    public void addPredecessor(TaskSymbolReference task) {
        this.predecessors.add(task);
    }

    public List<TaskSymbolReference> getPredecessors() {
        return Collections.unmodifiableList(this.predecessors);
    }

    public List<TaskSymbol> getPredecessorSymbols() {
        return this.getPredecessors().stream()
                .filter(TaskSymbolReference::existsReferencedSymbol)
                .map(TaskSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public void setFrontend(boolean frontend) {
        this.frontend = frontend;
    }

    public boolean isFrontend() {
        return this.frontend;
    }

    public void setBackend(boolean backend) {
        this.backend = backend;
    }

    public boolean isBackend() {
        return this.backend;
    }
}
