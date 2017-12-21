/**
 * ******************************************************************************
 * MontiCAR Modeling Family, www.se-rwth.de
 * Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 * All rights reserved.
 * <p>
 * This project is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3.0 of the License, or (at your option) any later version.
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 * <p>
 * You should have received a copy of the GNU Lesser General Public
 * License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.math.math._symboltable;

//import de.monticore.lang.math.math._ast.ASTMathStatement;

import de.monticore.lang.math.math._ast.ASTMathExpression;
import de.monticore.lang.math.math._ast.ASTMathStatements;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.symboltable.CommonSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathStatementsSymbol extends CommonSymbol {
    public static MathStatementsSymbolKind KIND = new MathStatementsSymbolKind();
    public ASTMathStatements astMathStatements = null;

    protected List<MathExpressionSymbol> mathExpressionSymbols = null;

    public MathStatementsSymbol(String name, ASTMathStatements ast) {
        super(name, KIND);
        this.astMathStatements = ast;
    }


    public ASTMathStatements getAstMathStatements() {
        return astMathStatements;
    }

    public void setAstMathStatements(ASTMathStatements astMathStatements) {
        this.astMathStatements = astMathStatements;
    }

    public List<MathExpressionSymbol> getMathExpressionSymbols() {
        if (mathExpressionSymbols == null) {
            mathExpressionSymbols = new ArrayList<>();
            for (ASTMathExpression astMathExpression : astMathStatements.getMathExpressions()) {
                mathExpressionSymbols.add((MathExpressionSymbol) astMathExpression.getSymbol().get());
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
            }
        }
    }

    public void addMathExpressionAfter(MathExpressionSymbol mathExpressionToAdd,
                                       MathExpressionSymbol referenceMathExpression) {
        for (int i = 0; i < mathExpressionSymbols.size(); ++i) {
            MathExpressionSymbol curExpression = mathExpressionSymbols.get(i);
            if (referenceMathExpression.equals(curExpression)) {
                mathExpressionSymbols.add(i + 1, mathExpressionToAdd);
            }
        }
    }
}
