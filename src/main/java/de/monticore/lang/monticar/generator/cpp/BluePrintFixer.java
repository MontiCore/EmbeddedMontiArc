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
package de.monticore.lang.monticar.generator.cpp;

import de.monticore.lang.monticar.generator.BluePrint;
import de.monticore.lang.monticar.generator.Variable;
import de.se_rwth.commons.logging.Log;

import java.util.*;
import java.util.stream.Collectors;

/**
 * @author Sascha Schneiders
 */
public class BluePrintFixer {
    public static void fixBluePrintVariableArrays(BluePrint bluePrint) {
        List<Variable> newVars = new ArrayList<>();

        //Group variables of the same array
        //Ports that are not part of an array are handled as array with size 1
        Map<String, List<Variable>> nameToVariable = bluePrint.getVariables().stream()
                .collect(Collectors.groupingBy(Variable::getNameWithoutArrayNamePart));

        //Used to keep the original order
        List<String> orderedUniqueNames = bluePrint.getVariables().stream()
                .map(Variable::getNameWithoutArrayNamePart)
                .distinct()
                .collect(Collectors.toList());

        //Only keep one and set the right array size
        orderedUniqueNames.forEach((nameWithoutArray) -> {
            List<Variable> varList = nameToVariable.get(nameWithoutArray);
            Variable firstVar = varList.get(0);
            firstVar.setName(nameWithoutArray);
            firstVar.setArraySize(varList.size());
            newVars.add(firstVar);
        });

        bluePrint.setVariables(newVars);
    }
}
