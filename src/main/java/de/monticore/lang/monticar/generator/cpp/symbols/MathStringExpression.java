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
import de.monticore.lang.math._symboltable.matrix.MathMatrixAccessSymbol;

import java.util.ArrayList;
import java.util.List;

/**
 * @author Sascha Schneiders
 */
public class MathStringExpression extends MathExpressionSymbol {
    public static int ID = 10001;

    protected String text;
    protected List<MathMatrixAccessSymbol> previousExpressionSymbols = new ArrayList<>();

    public MathStringExpression() {
        setID(ID);
    }

    public MathStringExpression(String text, List<MathMatrixAccessSymbol> previousExpressionSymbol) {
        this.text = text;
        setID(ID);
        if (previousExpressionSymbol != null)
            this.previousExpressionSymbols.addAll(previousExpressionSymbol);
    }

    public String getText() {
        return text;
    }

    public void setText(String text) {
        this.text = text;
    }

    @Override
    public String getTextualRepresentation() {
        return text;
    }

    public List<MathMatrixAccessSymbol> getPreviousExpressionSymbols() {
        return previousExpressionSymbols;
    }

}
