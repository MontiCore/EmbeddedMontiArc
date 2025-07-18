/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 * In AST: Name& = MathExpression
 *
 */
public class MathForLoopHeadSymbol extends MathExpressionSymbol {
    protected String nameLoopVariable;

    protected MathExpressionSymbol mathExpressionSymbol;

    public MathForLoopHeadSymbol() {

    }

    public String getNameLoopVariable() {
        return nameLoopVariable;
    }

    public void setNameLoopVariable(String nameLoopVariable) {
        this.nameLoopVariable = nameLoopVariable;
    }

    public MathExpressionSymbol getMathExpression() {
        return mathExpressionSymbol;
    }

    public void setMathExpression(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = mathExpressionSymbol;
    }

    @Override
    public String getTextualRepresentation() {
        return nameLoopVariable + "=" + mathExpressionSymbol.getTextualRepresentation();
    }



}
