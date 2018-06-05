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
package de.monticore.lang.embeddedmontiarc.embeddedmontiarcmath.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTElement;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTSubComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.math._ast.ASTMathStatements;
import de.se_rwth.commons.logging.Log;

/**
 * @author Sascha Schneiders
 */
public class AtomicComponentCoCo implements EmbeddedMontiArcASTComponentCoCo {
    /**
     * @see EmbeddedMontiArcASTComponentCoCo#check(ASTComponent)
     */
    @Override
    public void check(ASTComponent node) {
        boolean hasSubComponent = false;
        boolean hasImplementationMath = false;
        for (ASTElement element : node.getBody().getElementList()) {
            if (element instanceof ASTMathStatements) {
                hasImplementationMath = true;
            } else if (element instanceof ASTSubComponent) {
                hasSubComponent = true;
            }
        }
        if (hasImplementationMath && hasSubComponent) {
            Log.error("0x00000AE1 Implementation Math section in non atomic component " + node.getName() + " detected");
        }
    }

}
