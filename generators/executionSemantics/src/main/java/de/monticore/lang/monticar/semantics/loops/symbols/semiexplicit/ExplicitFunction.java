/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.loops.symbols.semiexplicit;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.math._symboltable.expression.MathArithmeticExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

public class ExplicitFunction implements EquationSystemFunction {

    private final EMAMEquationSymbol equation;
    private final MathArithmeticExpressionSymbol rightSideExpression;

    public ExplicitFunction(EMAMEquationSymbol equation) {
        this.equation = equation;
        rightSideExpression = new MathArithmeticExpressionSymbol();
        rightSideExpression.setOperator("-");
        rightSideExpression.setLeftExpression(equation.getLeftExpression());
        rightSideExpression.setRightExpression(equation.getRightExpression());
    }

    public EMAMEquationSymbol getEquation() {
        return equation;
    }

    public MathExpressionSymbol getAsRightSideExpression() {
        return rightSideExpression;
    }
}
