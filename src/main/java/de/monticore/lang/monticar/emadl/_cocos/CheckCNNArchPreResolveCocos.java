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

import de.monticore.lang.monticar.cnnarch._ast.ASTCNNArchNode;
import de.monticore.lang.monticar.cnnarch._cocos.CNNArchPreResolveCocos;
import de.monticore.lang.monticar.emadl._ast.ASTArchitectureConstructor;
import de.monticore.lang.monticar.emadl._symboltable.ArchitectureConstructorSymbol;
import de.monticore.lang.monticar.emadl.helper.ErrorCodes;
import de.se_rwth.commons.logging.Log;

public class CheckCNNArchPreResolveCocos implements EMADLASTArchitectureConstructorCoCo {

    @Override
    public void check(ASTArchitectureConstructor node) {
        ArchitectureConstructorSymbol sym = (ArchitectureConstructorSymbol) node.getSymbol().get();
        if (sym.getArchitecture() != null) {
            CNNArchPreResolveCocos.createChecker().checkAll((ASTCNNArchNode) sym.getArchitecture().getAstNode().get());
        }
        else {
            Log.error("0" + ErrorCodes.UNKNOWN_ARCHITECTURE_CODE + " Unknown architecture: " + node.getName() + ". " +
                            "The architecture has to be either in the same directory or imported."
                    , node.get_SourcePositionStart());
        }
    }
}
