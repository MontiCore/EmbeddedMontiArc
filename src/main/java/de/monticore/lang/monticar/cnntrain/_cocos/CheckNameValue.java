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
package de.monticore.lang.monticar.cnntrain._cocos;

import de.monticore.lang.monticar.cnntrain._ast.ASTVariableReference;
import de.monticore.lang.monticar.cnntrain._symboltable.ConfigParameterSymbol;
import de.monticore.lang.monticar.cnntrain._symboltable.NameValueSymbol;
import de.monticore.lang.monticar.cnntrain.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

import java.util.Collection;

public class CheckNameValue implements CNNTrainASTVariableReferenceCoCo {

    @Override
    public void check(ASTVariableReference node) {
        Collection<ConfigParameterSymbol> parameterCollection = node.getEnclosingScope().get().resolveMany(node.getName(), ConfigParameterSymbol.KIND);
        if (parameterCollection.isEmpty()){
            Log.error("0" + ErrorCodes.UNKNOWN_VARIABLE_CODE + " Unknown variable with name '" + node.getName() + "'."
                    , node.get_SourcePositionStart());
        }
        else if (parameterCollection.size() > 1){
            Log.error("0" + ErrorCodes.PARAMETER_REPETITION_CODE + " Parameter with name '" + node.getName() + "' is declared multiple times"
                    , node.get_SourcePositionStart());
        }
    }

}
