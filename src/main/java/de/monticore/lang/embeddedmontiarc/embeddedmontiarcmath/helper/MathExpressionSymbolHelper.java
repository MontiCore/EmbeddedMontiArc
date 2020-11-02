/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.helper;

import de.monticore.lang.math._symboltable.MathForLoopHeadSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.*;
import de.monticore.lang.math._symboltable.matrix.*;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolVisitor;

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

    private static class AddAllVisitor implements MathExpressionSymbolVisitor {
        List<MathExpressionSymbol> symbols = new LinkedList<>();

        @Override public void visit(MathArithmeticExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathAssignmentExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathBooleanExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathCompareExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathConditionalExpressionsSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathConditionalExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathForLoopHeadSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathForLoopExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathNameExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathNumberExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathParenthesisExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathPreOperatorExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathValueSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathValueType node) {
            symbols.add(node);
        }

        @Override public void visit(MathMatrixAccessOperatorSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathMatrixNameExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathMatrixVectorExpressionSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathMatrixArithmeticValueSymbol node) {
            symbols.add(node);
        }

        @Override public void visit(MathMatrixAccessSymbol node) {
            symbols.add(node);
        }

        protected Set<MathExpressionSymbol> visitedSymbols = new HashSet<>();

        @Override
        public Set<MathExpressionSymbol> getVisitedSymbols() {
            return visitedSymbols;
        }
    }
}
