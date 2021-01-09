/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.semantics.setup;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolReplacementVisitor;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathExpressionSymbolReplacementVisitor;

import java.util.Map;
import java.util.function.Function;

public class Instances {

    public static MathExpressionSymbolReplacementVisitor instantiateMathExpressionSymbolReplacement(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        return new EMAMMathExpressionSymbolReplacementVisitor(replacementMap);
    }

    public static MathExpressionSymbolReplacementVisitor instantiateMathExpressionSymbolReplacement(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        return new EMAMMathExpressionSymbolReplacementVisitor(replacementFunction);
    }

}
