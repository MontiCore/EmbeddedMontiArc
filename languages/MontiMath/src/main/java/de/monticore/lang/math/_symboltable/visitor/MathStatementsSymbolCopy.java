/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable.visitor;

import de.monticore.lang.math._ast.ASTMathStatements;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.List;

public class MathStatementsSymbolCopy extends MathStatementsSymbol {
    public MathStatementsSymbolCopy(String name, ASTMathStatements ast) {
        super(name, ast);
    }

    public void setMathExpressionSymbols(List<MathExpressionSymbol> mathExpressionSymbols) {
        this.mathExpressionSymbols = mathExpressionSymbols;
    }
}
