/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolParentAwareVisitor;
import de.monticore.lang.monticar.cnnarch._symboltable.TupleExpressionSymbol;

public interface EMADLMathExpressionSymbolParentAwareVisitor extends EMAMMathExpressionSymbolParentAwareVisitor,
        EMADLMathExpressionSymbolVisitor {

    @Override
    default void handle(TupleExpressionSymbol node) {
        if (shouldContinue(node)) {
            visit(node);
            pushParent(node);
            traverse(node);
            popParent();
            endVisit(node);
        }
    }
}
