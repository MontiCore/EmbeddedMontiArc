/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.mathopt._symboltable.MathOptimizationConditionSymbol;
import de.monticore.lang.mathopt._symboltable.MathOptimizationStatementSymbol;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolVisitor;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

public class MathExpressionSymbolHelper {

    public static List<MathExpressionSymbol> getAllSubExpressions(MathStatementsSymbol statements) {
        List<MathExpressionSymbol> symbols = new LinkedList<>();
        statements.getMathExpressionSymbols().stream().forEachOrdered(
                s -> symbols.addAll(getAllSubExpressions(s))
        );
        return symbols;
    }

    public static List<MathExpressionSymbol> getAllSubExpressions(MathExpressionSymbol symbol) {
        AddAllVisitor visitor = new AddAllVisitor();
        visitor.handle(symbol);
        return visitor.symbols;
    }

    private static class AddAllVisitor implements EMAMMathExpressionSymbolVisitor {
        List<MathExpressionSymbol> symbols = new LinkedList<>();

        @Override
        public void visit(MathArithmeticExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathAssignmentExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathBooleanExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathCompareExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathConditionalExpressionsSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathConditionalExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathForLoopHeadSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathForLoopExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathNameExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathNumberExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathParenthesisExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathPreOperatorExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathValueSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathValueType node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathMatrixAccessOperatorSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathMatrixNameExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathMatrixVectorExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathMatrixArithmeticValueSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathMatrixAccessSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathOptimizationStatementSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(MathOptimizationConditionSymbol node) {
            symbols.add(node);

        }

        @Override
        public void visit(MathMatrixArithmeticExpressionSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(EMAMSpecificationSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(EMAMSymbolicVariableSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(EMAMEquationSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(EMAMInitialGuessSymbol node) {
            symbols.add(node);
        }

        @Override
        public void visit(EMAMInitialValueSymbol node) {
            symbols.add(node);
        }

        protected Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return visitedSymbols;
        }
    }
}
