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
package de.monticore.lang.math._symboltable.matrix;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * @author Sascha Schneiders
 */
public class MathMatrixAccessOperatorSymbol extends MathMatrixExpressionSymbol {
    protected MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol;
    protected List<MathMatrixAccessSymbol> mathMatrixAccessSymbols = new ArrayList<>();
    protected String accessStartSymbol = "(";
    protected String accessEndSymbol = ")";

    public MathMatrixAccessOperatorSymbol() {
        super();
    }

    public MathMatrixNameExpressionSymbol getMathMatrixNameExpressionSymbol() {
        return mathMatrixNameExpressionSymbol;
    }

    public void setMathMatrixNameExpressionSymbol(MathMatrixNameExpressionSymbol mathMatrixNameExpressionSymbol) {
        this.mathMatrixNameExpressionSymbol = mathMatrixNameExpressionSymbol;
    }

    public String getAccessStartSymbol() {
        return accessStartSymbol;
    }

    public void setAccessStartSymbol(String accessStartSymbol) {
        this.accessStartSymbol = accessStartSymbol;
    }

    public String getAccessEndSymbol() {
        return accessEndSymbol;
    }

    public void setAccessEndSymbol(String accessEndSymbol) {
        this.accessEndSymbol = accessEndSymbol;
    }

    public boolean isDoubleDot(int index) {
        return mathMatrixAccessSymbols.get(index).isDoubleDot();
    }

    public Optional<MathExpressionSymbol> getMathMatrixAccessSymbol(int index) {
        return mathMatrixAccessSymbols.get(index).getMathExpressionSymbol();
    }

    public List<MathMatrixAccessSymbol> getMathMatrixAccessSymbols() {
        return mathMatrixAccessSymbols;
    }

    public void setMathMatrixAccessSymbols(List<MathMatrixAccessSymbol> mathMatrixAccessSymbols) {
        this.mathMatrixAccessSymbols = mathMatrixAccessSymbols;
    }

    public void addMathMatrixAccessSymbol(MathMatrixAccessSymbol symbol) {
        mathMatrixAccessSymbols.add(symbol);
    }

    @Override
    public String getTextualRepresentation() {
        String result = accessStartSymbol;
        int counter = 0;
        for (MathMatrixAccessSymbol accessSymbol : mathMatrixAccessSymbols) {
            result += accessSymbol.getTextualRepresentation();
            ++counter;
            if (counter < mathMatrixAccessSymbols.size())
                result += ", ";
        }
        result += accessEndSymbol;

        return result;
    }


}
