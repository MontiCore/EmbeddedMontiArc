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
package de.monticore.lang.monticar.emadl._cocos;

import de.monticore.lang.monticar.cnntrain._symboltable.ConfigParameterSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTConfigConstructor;
import de.monticore.lang.monticar.emadl._ast.ASTNamedArgument;
import de.monticore.lang.monticar.emadl._symboltable.ConfigConstructorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class CheckConfigArgument implements EMADLASTConfigConstructorCoCo {

    @Override
    public void check(ASTConfigConstructor node) {
        ConfigConstructorSymbol symbol = (ConfigConstructorSymbol) node.getSymbol().get();
        Set<ConfigParameterSymbol> remainingParameters = new HashSet<>(symbol.getConfiguration().getParameters());

        for (ASTNamedArgument argument : node.getArguments()){
            Optional<ConfigParameterSymbol> parameter = symbol.getConfiguration().getParameter(argument.getName());
            if (parameter.isPresent()){
                remainingParameters.remove(parameter.get());
            }
            else {
                Log.error("0"+ ErrorCodes.UNKNOWN_ARGUMENT_CODE + " Unknown Argument. " +
                                "Parameter with name '" + argument.getName() + "' does not exist. " +
                                "Existing parameters are: " + Joiners.COMMA.join(symbol.getConfiguration().getParameters())
                        , argument.get_SourcePositionStart());
            }
        }

        for (ConfigParameterSymbol remainingParam : remainingParameters){
            if (!remainingParam.getValue().isPresent()) {
                Log.error("0" + ErrorCodes.MISSING_ARGUMENT_CODE + " Missing argument. " +
                                "The argument '" + remainingParam.getName() + "' is required."
                        , node.get_SourcePositionStart());
            }
        }
    }
}
