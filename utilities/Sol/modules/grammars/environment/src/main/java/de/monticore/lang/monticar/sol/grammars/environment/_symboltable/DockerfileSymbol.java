/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import de.monticore.lang.monticar.sol.grammars.environment._ast.ASTExpose;

import java.util.*;
import java.util.stream.Collectors;

public class DockerfileSymbol extends DockerfileSymbolTOP {
    protected final List<DockerfileSymbolReference> components;

    protected boolean component;

    public DockerfileSymbol(String name) {
        super(name);

        this.components = new ArrayList<>();
    }

    public void setComponent(boolean component) {
        this.component = component;
    }

    public boolean isComponent() {
        return this.component;
    }

    public void addComponent(DockerfileSymbolReference component) {
        this.components.add(component);
    }

    public List<DockerfileSymbolReference> getComponents() {
        return Collections.unmodifiableList(this.components);
    }

    public List<DockerfileSymbol> getComponentSymbols() {
        return this.getComponents().stream()
                .filter(DockerfileSymbolReference::existsReferencedSymbol)
                .map(DockerfileSymbolReference::getReferencedSymbol)
                .collect(Collectors.toList());
    }

    public Set<Integer> getLocalPorts() {
        return this.getDockerfileNode()
                .map(node -> node.getInstructionList().stream()
                        .filter(instruction -> instruction instanceof ASTExpose)
                        .map(instruction -> (ASTExpose)instruction)
                        .map(expose -> expose.getPort().getValue())
                        .collect(Collectors.toSet())
                ).orElse(new HashSet<>());
    }

    public Set<Integer> getAllPorts() {
        Set<Integer> localPorts = this.getLocalPorts();
        Set<Integer> allPorts = new HashSet<>(localPorts);

        this.getComponentSymbols().forEach(symbol -> allPorts.addAll(symbol.getAllPorts()));
        return allPorts;
    }
}
