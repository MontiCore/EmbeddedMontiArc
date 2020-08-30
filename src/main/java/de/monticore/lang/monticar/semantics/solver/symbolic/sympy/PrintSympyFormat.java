/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.solver.symbolic.sympy;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;
import de.monticore.prettyprint.IndentPrinter;

public class PrintSympyFormat implements MathExpressionSymbolVisitor {

    IndentPrinter printer = new IndentPrinter();

    public static String print(MathExpressionSymbol node) {
        PrintSympyFormat print = new PrintSympyFormat();
        print.handle(node);
        return print.printer.getContent();
    }

    @Override public void visit(MathArithmeticExpressionSymbol node) {

    }

    @Override public void visit(MathAssignmentExpressionSymbol node) {

    }

    @Override public void visit(MathBooleanExpressionSymbol node) {

    }

    @Override public void visit(MathCompareExpressionSymbol node) {

    }

    @Override public void visit(MathConditionalExpressionsSymbol node) {

    }

    @Override public void visit(MathConditionalExpressionSymbol node) {

    }

    @Override public void visit(MathForLoopHeadSymbol node) {

    }

    @Override public void visit(MathForLoopExpressionSymbol node) {

    }

    @Override public void visit(MathNameExpressionSymbol node) {

    }

    @Override public void visit(MathNumberExpressionSymbol node) {

    }

    @Override public void visit(MathParenthesisExpressionSymbol node) {

    }

    @Override public void visit(MathPreOperatorExpressionSymbol node) {

    }

    @Override public void visit(MathValueSymbol node) {

    }

    @Override public void visit(MathValueType node) {

    }

    @Override public void visit(MathMatrixAccessOperatorSymbol node) {

    }

    @Override public void visit(MathMatrixNameExpressionSymbol node) {

    }

    @Override public void visit(MathMatrixVectorExpressionSymbol node) {

    }

    @Override public void visit(MathMatrixArithmeticValueSymbol node) {

    }

    @Override public void visit(MathMatrixAccessSymbol node) {

    }

    @Override public void visit(MathMatrixArithmeticExpressionSymbol node) {

    }
}
