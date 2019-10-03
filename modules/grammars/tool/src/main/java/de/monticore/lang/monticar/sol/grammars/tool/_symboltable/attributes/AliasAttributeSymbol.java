/*
 * (c) https://github.com/MontiCore/monticore
 */
package de.monticore.lang.monticar.sol.grammars.tool._symboltable.attributes;

import de.monticore.lang.monticar.sol.grammars.tool._ast.ToolLiterals;
import de.monticore.lang.monticar.sol.grammars.tool._symboltable.AttributeSymbol;

public class AliasAttributeSymbol extends AttributeSymbol {
    protected String alias;
    protected ToolLiterals registry;

    public AliasAttributeSymbol(String name) {
        super(name);
    }

    public void setAlias(String alias) {
        this.alias = alias;
    }

    public String getAlias() {
        return this.alias;
    }
}
