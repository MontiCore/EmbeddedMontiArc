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
package de.monticore.lang.math.math._symboltable.matrix;

import de.monticore.lang.math.math._ast.ASTMathMatrixNameExpression;
import de.monticore.lang.math.math._symboltable.expression.IMathNamedExpression;
import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;

import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathMatrixNameExpressionSymbol extends MathMatrixExpressionSymbol implements IMathNamedExpression {

    protected Optional<ASTMathMatrixNameExpression> astMathMatrixNameExpression = Optional.empty();
    protected String nameToAccess;
    protected Optional<MathMatrixAccessOperatorSymbol> mathMatrixAccessOperatorSymbol = Optional.empty();

    public MathMatrixNameExpressionSymbol(String nameToAccess) {
        super();
        this.nameToAccess = nameToAccess;
    }

    public boolean isASTMathMatrixNamePresent() {
        return astMathMatrixNameExpression.isPresent();
    }

    public ASTMathMatrixNameExpression getAstMathMatrixNameExpression() {
        return astMathMatrixNameExpression.get();
    }

    public void setAstMathMatrixNameExpression(ASTMathMatrixNameExpression astMathMatrixNameExpression) {
        this.astMathMatrixNameExpression = Optional.of(astMathMatrixNameExpression);
    }

    public String getNameToAccess() {
        return nameToAccess;
    }

    public void setNameToAccess(String nameToAccess) {
        this.nameToAccess = nameToAccess;
    }

    public boolean hasEndOperator() {
        return astMathMatrixNameExpression.get().getEndOperator().isPresent();
    }

    public boolean hasMatrixAccessExpression() {
        return astMathMatrixNameExpression.get().getMathMatrixAccessExpression().isPresent();
    }

    public MathMatrixAccessOperatorSymbol getMathMatrixAccessOperatorSymbol() {
        return mathMatrixAccessOperatorSymbol.get();
    }

    public boolean isMathMatrixAccessOperatorSymbolPresent() {
        return mathMatrixAccessOperatorSymbol.isPresent();
    }

    public void setMathMatrixAccessOperatorSymbol(MathMatrixAccessOperatorSymbol mathMatrixAccessOperatorSymbol) {
        if (astMathMatrixNameExpression.isPresent()) {
            astMathMatrixNameExpression.get().getMathMatrixAccessExpression().get().setSymbol(mathMatrixAccessOperatorSymbol);
        }
        this.mathMatrixAccessOperatorSymbol = Optional.of(mathMatrixAccessOperatorSymbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = "";
        result += nameToAccess;
        result += "(";
        if (isMathMatrixAccessOperatorSymbolPresent())
            result += getMathMatrixAccessOperatorSymbol().getTextualRepresentation();
        else if (isASTMathMatrixNamePresent() && astMathMatrixNameExpression.get().getEndOperator().isPresent()) {
            result += astMathMatrixNameExpression.get().getEndOperator().get().getSymbol();
        }
        result += ")";

        return result;
    }

    @Override
    public boolean isMatrixNameExpression() {
        return true;
    }


}
