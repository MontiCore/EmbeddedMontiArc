/* (c) https://github.com/MontiCore/monticore */


package de.monticore.lang.embeddedmontiarc.embeddedmontiarcapplication._symboltable;

import de.monticore.symboltable.SymbolKind;

import java.util.LinkedHashSet;
import java.util.Set;

public class EmbeddedMontiArcApplicationModelNameCalculator extends de.monticore.CommonModelNameCalculator {

  @Override
  public Set<String> calculateModelNames(final String name, final SymbolKind kind) {
    final Set<String> calculatedModelNames = new LinkedHashSet<>();

      if (EMAAplCompilationUnitSymbol.KIND.isKindOf(kind)) {
        calculatedModelNames.addAll(calculateModelNamesForEMAMCompilationUnit(name));
      }

    return calculatedModelNames;
  }

  protected Set<String> calculateModelNamesForEMAMCompilationUnit(String name) {
    final Set<String> modelNames = new LinkedHashSet<>();
    modelNames.add(name);
    return modelNames;
  }


}
