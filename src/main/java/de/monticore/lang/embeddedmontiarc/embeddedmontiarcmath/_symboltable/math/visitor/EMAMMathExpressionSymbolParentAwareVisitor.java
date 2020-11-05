/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMEquationSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialGuessSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.EMAMInitialValueSymbol;
import de.monticore.lang.mathopt._symboltable.visitor.MathOptExpressionSymbolParentAwareVisitor;

public interface EMAMMathExpressionSymbolParentAwareVisitor extends MathOptExpressionSymbolParentAwareVisitor,
    EMAMMathExpressionSymbolVisitor {

    @Override
    default void handle(EMAMEquationSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    default void handle(EMAMInitialGuessSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }

    @Override
    default void handle(EMAMInitialValueSymbol node) {
        pushParent(node);
        visit(node);
        traverse(node);
        endVisit(node);
        popParent();
    }
}
