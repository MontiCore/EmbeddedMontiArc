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
package de.monticore.lang.monticar.generator.cpp.symbols;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

/**
 * @author Sascha Schneiders
 */
public class MathChainedExpression extends MathExpressionSymbol {
    public static int ID = 10002;
    protected MathExpressionSymbol firstExpressionSymbol, secondExpressionSymbol;

    public MathChainedExpression() {
        setID(ID);
    }

    public MathChainedExpression(MathExpressionSymbol firstExpressionSymbol, MathExpressionSymbol secondExpressionSymbol) {
        this.firstExpressionSymbol = firstExpressionSymbol;
        this.secondExpressionSymbol = secondExpressionSymbol;
        setID(ID);
    }


    public MathExpressionSymbol getFirstExpressionSymbol() {
        return firstExpressionSymbol;
    }

    public void setFirstExpressionSymbol(MathExpressionSymbol firstExpressionSymbol) {
        this.firstExpressionSymbol = firstExpressionSymbol;
    }

    public MathExpressionSymbol getSecondExpressionSymbol() {
        return secondExpressionSymbol;
    }

    public void setSecondExpressionSymbol(MathExpressionSymbol secondExpressionSymbol) {
        this.secondExpressionSymbol = secondExpressionSymbol;
    }

    @Override
    public String getTextualRepresentation() {
        return firstExpressionSymbol.getTextualRepresentation() + secondExpressionSymbol.getTextualRepresentation();
    }

}
