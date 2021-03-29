/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl.math.visitor;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.symbols.*;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath._symboltable.math.visitor.CopyEMAMMathExpressionSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.visitor.MathStatementsSymbolCopy;
import de.monticore.lang.monticar.cnnarch._symboltable.TupleExpressionSymbol;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

public class CopyEMADLMathExpressionSymbol extends CopyEMAMMathExpressionSymbol
        implements EMADLMathExpressionSymbolVisitor {

    public static MathStatementsSymbol copy(MathStatementsSymbol symbol) {
        MathStatementsSymbolCopy res = new MathStatementsSymbolCopy(symbol.getName(), symbol.astMathStatements);
        if (symbol.getAstNode().isPresent()) res.setAstNode(symbol.getAstNode().get());
        res.setAccessModifier(symbol.getAccessModifier());
        res.setPackageName(symbol.getPackageName());
        res.setFullName(symbol.getFullName());
        List<MathExpressionSymbol> mathExpressionSymbolsCopy = new LinkedList<>();
        for (MathExpressionSymbol mathExpressionSymbol : symbol.getMathExpressionSymbols()) {
            mathExpressionSymbolsCopy.add(copy(mathExpressionSymbol));
        }
        res.setMathExpressionSymbols(mathExpressionSymbolsCopy);
        return res;
    }

    public static <T extends MathExpressionSymbol> T copy(T symbol) {
        CopyEMADLMathExpressionSymbol copy = new CopyEMADLMathExpressionSymbol();
        copy.handle(symbol);
        T res = copy.get(symbol);
        return res;
    }

    @Override
    protected <T extends MathExpressionSymbol> T get(T symbol) {
        MathExpressionSymbol copy = copyMap.get(symbol);
        if (copy != null) return (T) copy;
        if (symbol instanceof TupleExpressionSymbol)
            copy = new TupleExpressionSymbol();
        else
            return super.get(symbol);

        copyMap.put(symbol, copy);
        return (T) copy;
    }

    @Override
    public void endVisit(TupleExpressionSymbol node) {
        TupleExpressionSymbol res = get(node);
        copyMathExpressionSymbol(res, node);
        for (MathExpressionSymbol expression : node.getExpressions())
            res.add(get(expression));
    }
}
