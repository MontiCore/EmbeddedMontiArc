/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes;

import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbol;
import de.monticore.lang.monticar.sol.grammars.environment._symboltable.DockerfileSymbolReference;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;

import java.util.Optional;

public class EnvironmentAttributeSymbol extends AttributeSymbol {
    protected DockerfileSymbolReference environment;

    public EnvironmentAttributeSymbol(String name) {
        super(name);
    }

    public void setEnvironment(DockerfileSymbolReference environment) {
        this.environment = environment;
    }

    public Optional<DockerfileSymbolReference> getEnvironment() {
        return Optional.ofNullable(this.environment);
    }

    public Optional<DockerfileSymbol> getEnvironmentSymbol() {
        return this.getEnvironment().map(DockerfileSymbolReference::getReferencedSymbol);
    }
}
