/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.ast.ASTNode;
import de.monticore.lang.math._symboltable.expression.IArithmeticExpression;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 */
public class MathSymbolTableCreatorHelper {
    public static void setOperatorLeftRightExpression(IArithmeticExpression symbol, ASTNode leftExpressionSymbol, ASTNode rightExpressionSymbol, String operator) {
        if (rightExpressionSymbol != null) {
            setOperatorLeftRightExpression(symbol, (MathExpressionSymbol) leftExpressionSymbol.getSymbolOpt().get(), (MathExpressionSymbol) rightExpressionSymbol.getSymbolOpt().get(), operator);
        } else {
            setOperatorLeftRightExpression(symbol, (MathExpressionSymbol) leftExpressionSymbol.getSymbolOpt().get(), null, operator);
        }
    }

    public static void setOperatorLeftRightExpression(IArithmeticExpression symbol, MathExpressionSymbol leftExpressionSymbol, MathExpressionSymbol rightExpressionSymbol, String operator) {
        symbol.setOperator(operator);
        symbol.setLeftExpression(leftExpressionSymbol);
        symbol.setRightExpression(rightExpressionSymbol);
    }
}
