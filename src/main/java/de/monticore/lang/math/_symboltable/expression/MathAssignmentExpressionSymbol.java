/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.expression;

import de.monticore.lang.math._symboltable.MathAssignmentOperator;
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessOperatorSymbol;

/**
 * Symbol for setting/changing a MathValue
 *
 */
public class MathAssignmentExpressionSymbol extends MathExpressionSymbol {
    protected String nameOfMathValue;
    protected MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol;
    protected MathAssignmentOperator assignmentOperator;
    protected MathExpressionSymbol expressionSymbol;

    public String getNameOfMathValue() {
        return nameOfMathValue;
    }

    public void setNameOfMathValue(String nameOfMathValue) {
        this.nameOfMathValue = nameOfMathValue;
    }

    public MathMatrixAccessOperatorSymbol getMathMatrixAccessOperatorSymbol() {
        return mathMatrixAccessOperatorSymbol;
    }

    public void setMathMatrixAccessOperatorSymbol(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        this.mathMatrixAccessOperatorSymbol = mathMatrixAccessOperatorSymbol;
    }

    public MathAssignmentOperator getAssignmentOperator() {
        return assignmentOperator;
    }

    public void setAssignmentOperator(MathAssignmentOperator assignmentOperator) {
        this.assignmentOperator = assignmentOperator;
    }

    public MathExpressionSymbol getExpressionSymbol() {
        return expressionSymbol;
    }

    public void setExpressionSymbol(MathExpressionSymbol expressionSymbol) {
        this.expressionSymbol = expressionSymbol;
    }

    @Override
    public boolean isAssignmentExpression() {
        return true;
    }

    @Override
    public String getTextualRepresentation() {
        return nameOfMathValue + assignmentOperator.getOperator() + expressionSymbol.getTextualRepresentation() + ";";
    }


    @Override
    public MathExpressionSymbol getAssignedMathExpressionSymbol() {
        return expressionSymbol;
    }
}
