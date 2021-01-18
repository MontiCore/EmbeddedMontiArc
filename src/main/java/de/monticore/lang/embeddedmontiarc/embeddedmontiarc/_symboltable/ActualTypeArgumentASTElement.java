/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import de.monticore.symboltable.types.TypeSymbol;
import de.monticore.symboltable.types.references.ActualTypeArgument;
import de.monticore.symboltable.types.references.TypeReference;
import de.monticore.types.types._ast.ASTTypeArgument;

/**
 */
public class ActualTypeArgumentASTElement extends ActualTypeArgument {

    ASTTypeArgument astTypeArguments;

    public ActualTypeArgumentASTElement(boolean isLowerBound, boolean isUpperBound, TypeReference<? extends TypeSymbol> type) {
        super(isLowerBound, isUpperBound, type);
    }

    public ActualTypeArgumentASTElement(TypeReference<? extends TypeSymbol> type) {
        super(type);
    }

    public ASTTypeArgument getAstTypeArguments() {
        return astTypeArguments;
    }

    public ActualTypeArgumentASTElement setAstTypeArguments(ASTTypeArgument astTypeArguments) {
        this.astTypeArguments = astTypeArguments;
        return this;
    }
}
