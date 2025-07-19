/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.EMAMMathExpressionSymbolReplacementVisitor;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.TupleExpressionSymbol;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

public class EMADLMathExpressionSymbolReplacementVisitor extends EMAMMathExpressionSymbolReplacementVisitor
    implements EMADLMathExpressionSymbolVisitor {

    public EMADLMathExpressionSymbolReplacementVisitor(Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        super(replacementMap);
    }

    public EMADLMathExpressionSymbolReplacementVisitor(Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        super(replacementFunction);
    }

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        EMADLMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMADLMathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathExpressionSymbol mathExpressionSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        EMADLMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMADLMathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathExpressionSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Map<MathExpressionSymbol, MathExpressionSymbol> replacementMap) {
        EMADLMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMADLMathExpressionSymbolReplacementVisitor(replacementMap);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    public static void replace(MathStatementsSymbol mathStatementsSymbol,
                               Function<MathExpressionSymbol, MathExpressionSymbol> replacementFunction) {
        EMADLMathExpressionSymbolReplacementVisitor replacementVisitor =
                new EMADLMathExpressionSymbolReplacementVisitor(replacementFunction);
        replacementVisitor.handle(mathStatementsSymbol);
    }

    @Override
    public void visit(TupleExpressionSymbol node) {
        List<MathExpressionSymbol> expressions = new LinkedList();
        for (MathExpressionSymbol expression : node.getExpressions())
            expressions.add(get(expression));
        node.setExpressions(expressions);
    }
}
