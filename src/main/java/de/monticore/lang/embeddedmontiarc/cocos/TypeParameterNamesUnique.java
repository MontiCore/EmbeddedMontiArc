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
package de.monticore.lang.embeddedmontiarc.cocos;

import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo;
import de.monticore.lang.monticar.types2._ast.ASTTypeParameters2;
import de.monticore.lang.monticar.types2._ast.ASTTypeVariableDeclaration2;
import de.se_rwth.commons.logging.Log;

import java.util.ArrayList;
import java.util.List;

/**
 * @author (last commit) Crispin Kirchner
 */
public class TypeParameterNamesUnique implements EmbeddedMontiArcASTComponentCoCo {

    /**
     * @see de.monticore.lang.embeddedmontiarc.embeddedmontiarc._cocos.EmbeddedMontiArcASTComponentCoCo#check(de.monticore.lang.embeddedmontiarc.embeddedmontiarc._ast.ASTComponent)
     */
    @Override
    public void check(ASTComponent node) {
        if (node.getGenericTypeParametersOpt().isPresent()) {
            ASTTypeParameters2 typeParameters = node.getGenericTypeParametersOpt().get();
            List<String> typeParameterNames = new ArrayList<>();
            for (ASTTypeVariableDeclaration2 typeParameter : typeParameters.getTypeVariableDeclaration2List()) {

                if (typeParameter.getNamingResolutionOpt().isPresent() && typeParameterNames.contains(typeParameter.getNamingResolution().getName())) {
                    Log.error(String.format(
                            "0x35F1A The formal type parameter name \"%s\" is not unique",
                            typeParameter.getNamingResolution().getName()), typeParameter.get_SourcePositionStart());
                } else {
                    //typeParameterNames.add(typeParameter.getNamingResolution().get().getName());
                }
            }
        }
    }

}
