/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.sol.grammars.language._symboltable;

public class TemplateAttributeSymbol extends TemplateAttributeSymbolTOP {
    protected Object value;

    public TemplateAttributeSymbol(String name) {
        super(name);
    }

    public void setValue(Object value) {
        this.value = value;
    }

    public Object getValue() {
        return this.value;
    }
}
