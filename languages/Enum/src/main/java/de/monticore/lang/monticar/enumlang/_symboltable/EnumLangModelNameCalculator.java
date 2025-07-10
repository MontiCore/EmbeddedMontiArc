/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.enumlang._symboltable;

import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;
import de.se_rwth.commons.logging.Log;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class EnumLangModelNameCalculator extends EnumLangModelNameCalculatorTOP {

    @Override
    protected Set<String> calculateModelNamesForEnumConstantDeclaration(String name) {
        List<String> parts = Splitters.DOT.splitToList(name);
        if (parts.size() < 2) {
            Log.warn("suspicious enum name: " + name);
            return Collections.emptySet();
        }
        String modelName = Joiners.DOT.join(parts.subList(0, parts.size() - 1));
        HashSet<String> result = new HashSet<>();
        result.add(modelName);
        return result;
    }
}
