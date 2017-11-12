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

import de.monticore.lang.monticar.cnntrain._ast.ASTParameterAssignment;
import de.monticore.lang.monticar.cnntrain._ast.ASTTrainingConfiguration;
import de.se_rwth.commons.logging.Log;

import java.util.HashSet;
import java.util.Set;

public class DuplicatedParameterCheck implements CNNTrainASTTrainingConfigurationCoCo {

    @Override
    public void check(ASTTrainingConfiguration node) {
        Set<Enum> set = new HashSet<>();
        for (ASTParameterAssignment assignment : node.getAssignments()) {
            if (set.contains(assignment.getLhs())) {
                Log.error("0x03201 Multiple assignments of the same parameter are not allowed",
                        assignment.get_SourcePositionStart());
            }
            else {
                set.add(assignment.getLhs());
            }
        }
    }

}
