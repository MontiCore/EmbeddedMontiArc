/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.struct._symboltable;

import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;

import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class StructModelNameCalculator extends StructModelNameCalculatorTOP {
    @Override
    protected Set<String> calculateModelNamesForStructFieldDefinition(String name) {
        List<String> parts = Splitters.DOT.splitToList(name);
        if (parts.size() < 2) {
            return Collections.emptySet();
        }
        String modelName = Joiners.DOT.join(parts.subList(0, parts.size() - 1));
        HashSet<String> result = new HashSet<>();
        result.add(modelName);
        return result;
    }
}
