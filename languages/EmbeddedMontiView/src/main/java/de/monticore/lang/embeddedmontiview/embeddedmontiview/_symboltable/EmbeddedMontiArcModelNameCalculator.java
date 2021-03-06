/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.embeddedmontiview.embeddedmontiview._symboltable;

import com.google.common.collect.ImmutableSet;
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
    extends de.monticore.CommonModelNameCalculator {

  @Override
  public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
    final Set<String> calculatedModelNames = new LinkedHashSet<>();

    if (ViewComponentSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForComponent(name));
    }
    else if (ViewPortSymbol.KIND.isKindOf(kind) || ViewPortArraySymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForPort(name));
    }
    else if (ViewConnectorSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForConnector(name));
    }
    else if (ViewComponentInstanceSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForComponentInstance(name));
    }
    else if (ViewExpandedComponentInstanceSymbol.KIND.isKindOf(kind)) {
      calculatedModelNames.addAll(calculateModelNameForExpandedComponentInstance(name));
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
      return ImmutableSet.<String>builder().addAll(calculateModelNameForComponent(modelName)).addAll(calculateModelNameForExpandedComponentInstance(modelName)).build();
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

  protected Set<String> calculateModelNameForExpandedComponentInstance(String name) {
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
      return ImmutableSet.<String>builder().addAll(calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 1)))).addAll(calculateModelNameForComponent(Joiners.DOT.join(parts.subList(0, parts.size() - 2)))).addAll(calculateModelNameForExpandedComponentInstance(Joiners.DOT.join(parts.subList(0, parts.size() - 1)))).addAll(calculateModelNameForExpandedComponentInstance(Joiners.DOT.join(parts.subList(0, parts.size() - 2)))).build();
    }
    return ImmutableSet.of();
  }

}
