/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc.types;

import de.monticore.types.types._ast.ASTType;

/**
 */
public class EMAVariable {
    protected ASTType type;
    protected String name;

    public EMAVariable() {

    }

    public EMAVariable(ASTType type, String name) {
        this.type = type;
        this.name = name;
    }

    public ASTType getType() {
        return type;
    }

    public void setType(ASTType type) {
        this.type = type;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }
}
