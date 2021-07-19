package de.monticore.lang.gdl._ast;

public class ASTGameFunctionDefinition extends ASTGameFunctionDefinitionTOP {
    
    public ASTGameFunctionDefinition() {
        super();
    }

    public String getName() {
        return this.head.getName();
    }

}
