/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolVisitor;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.TupleExpressionSymbol;

public interface EMADLMathExpressionSymbolVisitor extends EMAMMathExpressionSymbolVisitor {

    @Override
    default void handle(MathExpressionSymbol node) {
        if (node == null) return;
        else if (node instanceof TupleExpressionSymbol)
            handle((TupleExpressionSymbol) node);
        else
            EMAMMathExpressionSymbolVisitor.super.handle(node);
    }

    public default void handle(TupleExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            traverse(node);
            endVisit(node);
        }
    }

    public default void traverse(TupleExpressionSymbol node) {
        for (MathExpressionSymbol expression : node.getExpressions()) {
            handle(expression);
        }
    }

    public default void visit(TupleExpressionSymbol node) {

    }

    public default void endVisit(TupleExpressionSymbol node) {

    }
}
