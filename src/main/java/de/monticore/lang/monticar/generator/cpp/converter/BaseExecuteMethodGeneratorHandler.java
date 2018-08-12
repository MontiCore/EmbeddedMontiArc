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
package de.monticore.lang.monticar.generator.cpp.converter;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.monticar.pattern.BaseChainOfResponsibility;
import de.se_rwth.commons.logging.Log;

import java.util.List;

/**
 * Implements the chain-of-responsibility pattern
 *
 * @author Christoph Richter
 */
public abstract class BaseExecuteMethodGeneratorHandler extends BaseChainOfResponsibility<BaseExecuteMethodGeneratorHandler> {

    protected abstract boolean canHandleSymbol(MathExpressionSymbol symbol);

    protected abstract String doGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings);

    public String handleGenerateExecuteCode(MathExpressionSymbol symbol, List<String> includeStrings) {
        String result = "";
        if (canHandleSymbol(symbol)) {
            result = doGenerateExecuteCode(symbol, includeStrings);
        } else if (getSuccessor() != null) {
            result = getSuccessor().handleGenerateExecuteCode(symbol, includeStrings);
        } else {
            Log.info(symbol.getTextualRepresentation(), "Symbol:");
            Log.debug(getRole(), "Case not handled!");
        }
        return result;
    }

}
