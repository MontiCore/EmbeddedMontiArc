/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes;

import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;

public class LiteralAttributeSymbol extends AttributeSymbol {
    protected Object value;

    public LiteralAttributeSymbol(String name) {
        super(name);
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Object getValue() {
        return this.value;
    }
}
