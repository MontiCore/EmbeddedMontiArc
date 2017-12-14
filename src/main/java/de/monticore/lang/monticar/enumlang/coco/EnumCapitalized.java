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
package de.monticore.lang.monticar.enumlang.coco;

import de.monticore.lang.monticar.enumlang._ast.ASTEnumDeclaration;
import de.monticore.lang.monticar.enumlang._cocos.EnumLangASTEnumDeclarationCoCo;
import de.se_rwth.commons.logging.Log;

public class EnumCapitalized implements EnumLangASTEnumDeclarationCoCo {

    @Override
    public void check(ASTEnumDeclaration node) {
        if (!Character.isUpperCase(node.getName().charAt(0))) {
            Log.error(
                    "Enum names must start with upper-case",
                    node.get_SourcePositionStart()
            );
        }
    }
}
