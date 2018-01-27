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

import de.monticore.lang.monticar.cnnarch._symboltable.VariableSymbol;
import de.monticore.lang.monticar.emadl._ast.ASTArchitectureConstructor;
import de.monticore.lang.monticar.emadl._ast.ASTNamedArgument;
import de.monticore.lang.monticar.emadl._symboltable.ArchitectureConstructorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class CheckArchitectureArgument implements EMADLASTArchitectureConstructorCoCo {

    @Override
    public void check(ASTArchitectureConstructor node) {
        ArchitectureConstructorSymbol symbol = (ArchitectureConstructorSymbol) node.getSymbol().get();
        Set<VariableSymbol> remainingParameters = new HashSet<>(symbol.getArchitecture().getParameters());

        for (ASTNamedArgument argument : node.getArguments()){
            Optional<VariableSymbol> parameter = symbol.getArchitecture().getParameter(argument.getName());
            if (parameter.isPresent()){
                remainingParameters.remove(parameter.get());
            }
            else {
                Log.error("0"+ ErrorCodes.UNKNOWN_ARGUMENT_CODE + " Unknown Argument. " +
                                "Parameter with name '" + argument.getName() + "' does not exist. " +
                                "Existing parameters are: " + Joiners.COMMA.join(symbol.getArchitecture().getParameters())
                        , argument.get_SourcePositionStart());
            }
        }

        for (VariableSymbol remainingParam : remainingParameters){
            if (!remainingParam.getDefaultExpression().isPresent()) {
                Log.error("0" + ErrorCodes.MISSING_ARGUMENT_CODE + " Missing argument. " +
                                "The argument '" + remainingParam.getName() + "' is required."
                        , node.get_SourcePositionStart());
            }
        }

    }

}
