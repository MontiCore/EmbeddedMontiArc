/*
 * Copyright (C) 2019 SE RWTH.
 *
 *  TODO: Include License.
 */
package de.monticore.lang.monticar.sol.grammars.options._symboltable;

public class OptionPropSymbol extends OptionPropSymbolTOP {
    protected Object value;

    public OptionPropSymbol(String name) {
        super(name);
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Object getValue() {
        return this.value;
    }
}
