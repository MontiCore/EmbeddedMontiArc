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

import de.monticore.lang.math.math._symboltable.expression.MathExpressionSymbol;

import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathMatrixAccessSymbol extends MathMatrixExpressionSymbol {

    protected Optional<MathExpressionSymbol> mathExpressionSymbol = Optional.empty();

    public MathMatrixAccessSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = Optional.of(mathExpressionSymbol);
    }

    //: Access per default
    public MathMatrixAccessSymbol() {
        super();
    }


    @Override
    public String getTextualRepresentation() {
        String result = "";
        if (mathExpressionSymbol.isPresent()) {
            result += mathExpressionSymbol.get().getTextualRepresentation();
        } else {
            result += ":";
        }
        return result;
    }

    public Optional<MathExpressionSymbol> getMathExpressionSymbol() {
        return mathExpressionSymbol;
    }

    public void setMathExpressionSymbol(MathExpressionSymbol mathExpressionSymbol) {
        this.mathExpressionSymbol = Optional.ofNullable(mathExpressionSymbol);
    }

    public boolean isDoubleDot() {
        return !mathExpressionSymbol.isPresent();
    }

    @Override
    public boolean isMatrixAccessExpression() {
        return true;
    }
}
