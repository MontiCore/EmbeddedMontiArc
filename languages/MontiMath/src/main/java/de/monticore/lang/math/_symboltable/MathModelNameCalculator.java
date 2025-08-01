/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.math._symboltable;

import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;

import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 */
public class MathModelNameCalculator extends de.monticore.CommonModelNameCalculator {

    @Override
    public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
        final Set<String> calculatedModelNames = new LinkedHashSet<>();

        /*if (AssignmentSymbol.KIND.isKindOf(kind)) {
            calculatedModelNames.addAll(calculateModelNamesForAssignment(name));
        }

        if (MathVariableDeclarationSymbol.KIND.isKindOf(kind)) {
            List<String> parts = Splitters.DOT.splitToList(name);
            String modelName = Joiners.DOT.join(parts.subList(0, parts.size() - 1));
            return ImmutableSet.of(modelName);
        }*/


        List<String> parts = Splitters.DOT.splitToList(name);
        String modelName = Joiners.DOT.join(parts.subList(0, parts.size() - 1));
        if (!modelName.isEmpty())
            calculatedModelNames.add(modelName);
        else
            calculatedModelNames.add(name);
        return calculatedModelNames;
    }

    protected Set<String> calculateModelNamesForAssignment(String name) {
        final Set<String> modelNames = new LinkedHashSet<>();
        modelNames.add(name);
        return modelNames;
    }
}
