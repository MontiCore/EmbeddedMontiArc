/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.option._symboltable;

import de.monticore.lang.monticar.sol.grammars.common._symboltable.SymbolWithType;

import java.util.Optional;

public class PropDeclarationSymbol extends PropDeclarationSymbolTOP implements SymbolWithType {
    protected boolean required;
    protected String type;

    public PropDeclarationSymbol(String name) {
        super(name);

        this.required = false;
    }

    public boolean isRequired() {
        return this.required;
    }

    public void setRequired(boolean required) {
        this.required = required;
    }

    @Override
    public void setType(String type) {
        this.type = type;
    }

    @Override
    public Optional<String> getType() {
        return Optional.ofNullable(this.type);
    }
}
