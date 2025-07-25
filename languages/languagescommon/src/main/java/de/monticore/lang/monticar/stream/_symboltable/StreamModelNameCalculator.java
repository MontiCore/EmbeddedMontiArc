/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.stream._symboltable;

import com.google.common.collect.ImmutableSet;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;

import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Helps loading inner components, by mapping their full-qualified names to the full-qualified name
 * of the most outer component of the file the inner one is defined in. This way the SymTab knows
 * which file to load. By convention, package names must be lower-case (see {@link PackageLowerCase}
 * ) and component names must startVal upper-case (see {@link ComponentCapitalized}). This ensures,
 * that we can calculate the most outer component, by searching for the first upper-case part of a
 * full-qualified name, e.g.:<br/>
 * a.b.C.D.E -> a.b.C
 *
 */
public class StreamModelNameCalculator
        extends de.monticore.CommonModelNameCalculator {

    @Override
    public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
        final Set<String> calculatedModelNames = new LinkedHashSet<>();

        if (ComponentStreamSymbol.KIND.isKindOf(kind)) {
            calculatedModelNames.addAll(calculateModelNameForComponent(name));
        } else if (NamedStreamSymbol.KIND.isKindOf(kind)) {
            calculatedModelNames.addAll(calculateModelNameForPort(name));
        }

        return calculatedModelNames;
    }

    protected Set<String> calculateModelNameForComponent(String name) {
        List<String> parts = Splitters.DOT.splitToList(name);
        Set<String> ret = new LinkedHashSet<>();

        for (int i = 0; i < parts.size(); i++) {
            char[] c = parts.get(i).toCharArray();
            if (Character.isUpperCase(c[0])) {
                ret.add(Joiners.DOT.join(parts.subList(0, i + 1)));
            }
        }

        return Collections.unmodifiableSet(ret);
    }

    protected Set<String> calculateModelNameForPort(String name) {
        List<String> parts = Splitters.DOT.splitToList(name);
        if (parts.size() > 1) {
            String modelName = Joiners.DOT.join(parts.subList(0, parts.size() - 1));
            return ImmutableSet.<String>builder()
                    .addAll(calculateModelNameForComponent(modelName))
                    .build();
        }
        return ImmutableSet.of();
    }


}
