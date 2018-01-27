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

import de.monticore.lang.monticar.cnntrain._ast.ASTCNNTrainNode;
import de.monticore.lang.monticar.cnntrain._cocos.CNNTrainCocos;
import de.monticore.lang.monticar.emadl._ast.ASTConfigConstructor;
import de.monticore.lang.monticar.emadl._symboltable.ConfigConstructorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckCNNTrainCocos implements EMADLASTConfigConstructorCoCo {

    @Override
    public void check(ASTConfigConstructor node) {
        ConfigConstructorSymbol sym = (ConfigConstructorSymbol) node.getSymbol().get();
        if (sym.getConfiguration() != null) {
            CNNTrainCocos.createChecker().checkAll((ASTCNNTrainNode) sym.getConfiguration().getAstNode().get());
        }
        else {
            Log.error("0" + ErrorCodes.UNKNOWN_CONFIGURATION_CODE + " Unknown configuration: " + node.getName() + ". " +
                    "The configuration has to be either in the same directory or imported."
                    , node.get_SourcePositionStart());
        }
    }
}
