/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.environment._symboltable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
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
}
