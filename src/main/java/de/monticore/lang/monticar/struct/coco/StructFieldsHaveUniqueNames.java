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
package de.monticore.lang.monticar.struct.coco;

import de.monticore.lang.monticar.struct._ast.ASTStruct;
import de.monticore.lang.monticar.struct._ast.ASTStructFieldDefinition;
import de.monticore.lang.monticar.struct._cocos.StructASTStructCoCo;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StructFieldsHaveUniqueNames implements StructASTStructCoCo {
    @Override
    public void check(ASTStruct node) {
        Set<String> fieldNames = new HashSet<>();
        List<ASTStructFieldDefinition> structFieldDefinitions = node.getStructFieldDefinitions();
        for (ASTStructFieldDefinition field : structFieldDefinitions) {
            if (!fieldNames.add(field.getName())) {
                Log.error(
                        "Duplicate field",
                        field.get_SourcePositionStart()
                );
            }
        }
    }
}
