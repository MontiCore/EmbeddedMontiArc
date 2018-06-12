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
