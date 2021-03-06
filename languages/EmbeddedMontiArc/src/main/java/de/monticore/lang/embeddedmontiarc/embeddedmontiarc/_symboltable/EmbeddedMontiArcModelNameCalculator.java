/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable;

import com.google.common.collect.ImmutableSet;
import de.monticore.CommonModelNameCalculator;
import de.monticore.lang.embeddedmontiarc.cocos.ComponentCapitalized;
import de.monticore.lang.embeddedmontiarc.cocos.PackageLowerCase;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAComponentSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAConnectorSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortArraySymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.cncModel.EMAPortSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstanceSymbol;
import de.monticore.lang.embeddedmontiarc.embeddedmontiarc._symboltable.instanceStructure.EMAComponentInstantiationSymbol;
import de.monticore.symboltable.SymbolKind;
import de.se_rwth.commons.Joiners;
import de.se_rwth.commons.Splitters;

import java.util.*;

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
public class EmbeddedMontiArcModelNameCalculator
    extends CommonModelNameCalculator {

  @Override
  public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
    final Set<String> calculatedModelNames = new LinkedHashSet<>();

    if (EMAComponentSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForComponent(name));
    }
    else if (EMAPortSymbol.KIND.isKindOf(kind) ||
            EMAPortArraySymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForPort(name));
    }
    else if (EMAConnectorSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForConnector(name));
    }
    else if (EMAComponentInstantiationSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForComponentInstance(name));
    }
    else if (EMAComponentInstanceSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForEMAComponentInstance(name));
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
          .addAll(calculateModelNameForEMAComponentInstance(modelName))
          .build();
    }
    return ImmutableSet.of();
  }

  protected Set<String> calculateModelNameForComponentInstance(String name) {
    List<String> parts = Splitters.DOT.splitToList(name);
    if (parts.size() > 1) {
      return calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 1)));
    }
    return ImmutableSet.of();
  }

  protected Set<String> calculateModelNameForEMAComponentInstance(String name) {
    List<String> parts = Splitters.DOT.splitToList(name);
    // adds all combinations since instances begin with a small capital latter
    // as packages do also
    Set<String> ret = new LinkedHashSet<>();
    for (int j = 0; j < parts.size(); j++) {
      ArrayList<String> tmp = new ArrayList<>(parts.subList(0, j));
      String s = parts.get(j);
      if (s.length() > 1) {
        tmp.add(Character.toUpperCase(s.charAt(0)) + s.substring(1));
      }
      else {
        tmp.add(Character.toUpperCase(s.charAt(0)) + "");
      }
      ret.add(Joiners.DOT.join(tmp));
    }

    return Collections.unmodifiableSet(ret);
  }

  protected Set<String> calculateModelNameForConnector(String name) {
    List<String> parts = Splitters.DOT.splitToList(name);
    if (parts.size() == 1) {
      return calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 1)));
    }
    else if (parts.size() >= 2) {
      return ImmutableSet.<String>builder()
          .addAll(calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 1))))
          .addAll(calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 2))))
          .addAll(calculateModelNameForEMAComponentInstance(Joiners.DOT.join(parts.subList(0, parts.size() - 1))))
          .addAll(calculateModelNameForEMAComponentInstance(Joiners.DOT.join(parts.subList(0, parts.size() - 2))))
          .build();
    }
    return ImmutableSet.of();
  }

}
