/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;
import java.util.List;


public class EMADLMathStatementsSymbol extends MathStatementsSymbol {
    protected List<MathExpressionSymbol> emadlMathExpressionSymbols = null;

    public EMADLMathStatementsSymbol(String name, List<ASTStatement> statements) {
        super(name, null);
        this.emadlMathExpressionSymbols = new ArrayList<>();
        for (ASTStatement astStatement : statements) {
            emadlMathExpressionSymbols.add((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        }
    }

    @Override
    public List<MathExpressionSymbol> getMathExpressionSymbols() {
        return this.emadlMathExpressionSymbols;
    }


}
