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
package de.monticore.lang.monticar.cnnarch._cocos;

import de.monticore.lang.math._symboltable.expression.MathExpressionSymbol;
import de.monticore.lang.math._symboltable.expression.MathNameExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ArchSimpleExpressionSymbol;
import de.monticore.lang.monticar.cnnarch._symboltable.ParameterSymbol;
import de.monticore.lang.monticar.cnnarch.helper.ErrorCodes;
import de.monticore.lang.monticar.cnnarch.helper.Utils;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class CheckExpressions extends CNNArchSymbolCoCo {

    @Override
    public void check(ArchExpressionSymbol sym) {
        if (sym instanceof ArchSimpleExpressionSymbol){
            checkNamesInExpression((ArchSimpleExpressionSymbol) sym);
        }
    }

    public void checkNamesInExpression(ArchSimpleExpressionSymbol expression) {
        if (expression.getMathExpression().isPresent()){
            MathExpressionSymbol mathExpression = expression.getMathExpression().get();

            for (MathExpressionSymbol subMathExp : Utils.createSubExpressionList(mathExpression)){
                if (subMathExp instanceof MathNameExpressionSymbol){
                    String name = ((MathNameExpressionSymbol) subMathExp).getNameToAccess();
                    Collection<ParameterSymbol> parameterCollection = expression.getEnclosingScope().resolveMany(name, ParameterSymbol.KIND);

                    if (parameterCollection.isEmpty()){
                        Log.error("0" + ErrorCodes.UNKNOWN_PARAMETER_NAME + " Unknown parameter name. " +
                                "The parameter '" + name + "' does not exist. "
                                , subMathExp.getSourcePosition());
                    }
                }
            }
        }
    }
}
