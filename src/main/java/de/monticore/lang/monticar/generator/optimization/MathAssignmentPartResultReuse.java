package de.monticore.lang.monticar.generator.optimization;

import de.monticore.lang.math.math._symboltable.expression.MathAssignmentExpressionSymbol;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;

import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathAssignmentPartResultReuse implements MathOptimizationRule {

    @Override
    public void optimize(MathExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {
        if (mathExpressionSymbol.isAssignmentExpression()) {
            optimize((MathAssignmentExpressionSymbol) mathExpressionSymbol, precedingExpressions);
        }
    }

    public void optimize(MathAssignmentExpressionSymbol mathExpressionSymbol, List<MathExpressionSymbol> precedingExpressions) {

    }
}
