/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

//import de.monticore.lang.math._ast.ASTMathStatement;

import de.monticore.expressionsbasis._ast.ASTExpression;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.CommonSymbol;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 */
public class MathStatementsSymbol extends CommonSymbol {
    public static MathStatementsSymbolKind KIND = new MathStatementsSymbolKind();
    public ASTMathStatements astMathStatements = null;

    protected List<MathExpressionSymbol> mathExpressionSymbols = null;

    public MathStatementsSymbol(String name, ASTMathStatements ast) {
        super(name, KIND);
        this.astMathStatements = ast;
    }


    public List<MathExpressionSymbol> getMathExpressionSymbols() {
        if (mathExpressionSymbols == null) {
            mathExpressionSymbols = new ArrayList<>();
            for (ASTStatement astStatement : astMathStatements.getStatementList()) {
                mathExpressionSymbols.add((MathExpressionSymbol) astStatement.getSymbolOpt().get());
            }
        }
        return mathExpressionSymbols;
    }

    public void addMathExpressionBefore(MathExpressionSymbol mathExpressionToAdd,
                                        MathExpressionSymbol referenceMathExpression) {
        for (int i = 0; i < mathExpressionSymbols.size(); ++i) {
            MathExpressionSymbol curExpression = mathExpressionSymbols.get(i);
            if (referenceMathExpression.equals(curExpression)) {
                mathExpressionSymbols.add(i, mathExpressionToAdd);
                Log.debug(mathExpressionToAdd.getTextualRepresentation(), "addMathExpressionBefore:");
                return;
            }
        }
    }

    public void addMathExpressionAfter(MathExpressionSymbol mathExpressionToAdd,
                                       MathExpressionSymbol referenceMathExpression) {
        for (int i = 0; i < mathExpressionSymbols.size(); ++i) {
            MathExpressionSymbol curExpression = mathExpressionSymbols.get(i);
            if (referenceMathExpression.equals(curExpression)) {
                mathExpressionSymbols.add(i + 1, mathExpressionToAdd);
                return;
            }
        }
    }

    public void replaceMathExpression(MathExpressionSymbol newMathExpression, MathExpressionSymbol oldMathExpressionSymbol) {
        MathExpressionReplacer.replaceMathExpression(this, newMathExpression, oldMathExpressionSymbol);
    }
}
