/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
package de.monticore.lang.monticar.emadl._symboltable;

import de.monticore.lang.math._ast.ASTStatement;
import de.monticore.lang.math._symboltable.MathStatementsSymbol;
import de.monticore.lang.math._symboltable.MathStatementsSymbolKind;
import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTMathStatements;

import java.util.ArrayList;
import java.util.List;


public class EMADLMathStatementsSymbol extends MathStatementsSymbol {
    protected List<MathExpressionSymbol> emadlMathExpressionSymbols = null;

    public EMADLMathStatementsSymbol(String name, ASTMathStatements ast) {
        super(name, null);
        this.emadlMathExpressionSymbols = new ArrayList<>();
        for (ASTStatement astStatement : ast.getStatementList()) {
            emadlMathExpressionSymbols.add((MathExpressionSymbol) astStatement.getSymbolOpt().get());
        }
    }

    @Override
    public List<MathExpressionSymbol> getMathExpressionSymbols() {
        return this.emadlMathExpressionSymbols;
    }


}
