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
package de.monticore.lang.monticar.generator;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.generator.cpp.BluePrintCPP;
import de.monticore.lang.monticar.pattern.BaseChainOfResponsibility;
import de.se_rwth.commons.logging.Log;

public abstract class BaseMathFunctionFixerHandler extends BaseChainOfResponsibility<BaseMathFunctionFixerHandler> {

    protected abstract boolean canFixMathSymbol(MathExpressionSymbol symbol);

    protected abstract void doFixMathFunction(MathExpressionSymbol symbol, BluePrintCPP bluePrintCPP);

    private void handleFixMathFunction(MathExpressionSymbol symbol, BluePrintCPP bluePrintCPP) {
        if (canFixMathSymbol(symbol)) {
            doFixMathFunction(symbol, bluePrintCPP);
        } else if (getSuccessor() != null) {
            ((BaseMathFunctionFixerHandler) getSuccessor()).handleFixMathFunction(symbol, bluePrintCPP);
        } else {
            Log.info(symbol.getTextualRepresentation(), "Symbol:");
            Log.debug(getRole(), "Case not handled!");
        }
    }

    public void chainHandleFixMathFunction(MathExpressionSymbol symbol, BluePrintCPP bluePrintCPP) {
        ((BaseMathFunctionFixerHandler) getChainStart()).handleFixMathFunction(symbol, bluePrintCPP);
    }

}
